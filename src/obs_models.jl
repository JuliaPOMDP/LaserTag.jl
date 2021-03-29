struct LTObsDist{M<:ObsModel}
    distances::ClearDistances
    model::M
end

obs_model(d::LTObsDist) = d.model

struct DESPOTEmu <: ObsModel
    std::Float64
    cdf::ReadingCDF
end

obs_type(::Type{DESPOTEmu}) = DMeas

gausscdf(mu, sigma, x) = (1+erf((x-mu)/(sigma*sqrt(2))))/2

function DESPOTEmu(f::Floor, std::Float64, maxread::Int=ceil(Int, max_diag(f)))
    maxclear = max(f.n_rows, f.n_cols) - 1
    cardcdf = Array{Float64}(undef, maxclear + 1, maxread + 1)

    for dist in 1:maxclear+1 # "dist = LaserRange" from cpp
        cdf = 0.0
        for reading in 0:maxread
            if reading < dist
                min_noise = reading - dist
                max_noise = min(dist, reading + 1) - dist
                cdf += 2*(gausscdf(0, std, max_noise)
                          - (reading > 0 ? gausscdf(0, std, min_noise) : 0.0))
                cardcdf[dist, reading+1] = cdf
            else
                cardcdf[dist, reading+1] = cdf
            end
        end
    end

    maxclear = min(f.n_rows, f.n_cols) - 1
    diagcdf = Array{Float64}(undef, maxclear + 1, maxread + 1)

    for c in 0:maxclear
        dist = sqrt(2)*(c+1)
        cdf = 0.0
        for reading in 0:maxread
            if reading < dist
                min_noise = reading - dist
                max_noise = min(dist, reading + 1) - dist
                cdf += 2*(gausscdf(0, std, max_noise)
                          - (reading > 0 ? gausscdf(0, std, min_noise) : 0.0))
                diagcdf[c+1, reading+1] = cdf
            else
                diagcdf[c+1, reading+1] = cdf
            end
        end
    end

    for c in 0:maxclear
        @assert abs(cardcdf[c+1, 1] + sum(diff(cardcdf[c+1, :])) - 1.0) < 1e-5 "cardinal cdf was $cardcdf"
        @assert abs(diagcdf[c+1, 1] + sum(diff(diagcdf[c+1, :])) - 1.0) < 1e-5 "diagonal cdf was $diagcdf"
    end

    return DESPOTEmu(std, ReadingCDF(cardcdf, diagcdf))
end

function Random.rand(rng::Random.AbstractRNG, d::LTObsDist{DESPOTEmu})
    if d.distances == CD_SAME_LOC
        return D_SAME_LOC
    end
    meas = MVector{8, Int}(undef)
    for i in 1:4
        meas[i] = max(0, floor(Int, n_clear_cells(d.distances, i)+1 - abs(d.model.std*randn(rng))))
    end
    for i in 5:8
        meas[i] = max(0, floor(Int, (n_clear_cells(d.distances, i)+1)*sqrt(2) - abs(d.model.std*randn(rng))))
    end
    return meas
end

function Distributions.pdf(d::LTObsDist{DESPOTEmu}, m::DMeas)
    a = d.distances == CD_SAME_LOC
    b = m == D_SAME_LOC
    if a || b
        return convert(Float64, a && b)
    end
    p = 1.0
    model = obs_model(d)
    for dir in 1:8
        if m[dir] == 0
            p *= cdf(model.cdf, dir, n_clear_cells(d.distances, dir), 0)
        else
            p *= (cdf(model.cdf, dir, n_clear_cells(d.distances, dir), m[dir]) - 
                  cdf(model.cdf, dir, n_clear_cells(d.distances, dir), m[dir]-1))
        end
    end
    return p
end

function POMDPs.observation(p::LaserTagPOMDP{M}, sp::LTState) where M
    return LTObsDist(p.dcache[sp], p.obs_model)
end
