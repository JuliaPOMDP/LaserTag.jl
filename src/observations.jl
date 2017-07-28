immutable CLTObsDist
    same::Bool
    distances::ClearDistances
    std::Float64

    CLTObsDist(same::Bool) = new(true)
    CLTObsDist(dists::ClearDistances, std::Float64) = new(false, dists, std)
end

function rand(rng::AbstractRNG, d::CLTObsDist)
    if d.same
        return C_SAME_LOC
    end
    meas = MVector{8, Float64}()
    meas[1:4] = d.distances.cardinal + d.std*randn(rng, 4)
    meas[5:8] = d.distances.diagonal*sqrt(2) + d.std*randn(rng, 4)
    return meas
end

function pdf(d::CLTObsDist, m::CMeas)
    if d.same
        return m == C_SAME_LOC ? 1.0 : 0.0
    elseif m == C_SAME_LOC
        return 0.0
    end
    diff = m - vcat(float(d.distances.cardinal), d.distances.diagonal*sqrt(2))
    return exp(sum(-diff.^2/(2*d.std^2)))
end

function observation(p::LaserTagPOMDP{CMeas}, sp::LTState)
    if sp.robot == sp.opponent
        return CLTObsDist(true)
    end
    distances = p.dcache[sp]
    return CLTObsDist(distances, p.reading_std)
end

# to generate a reading from this, take the clear distance, add sample from a gaussian, and round to the nearest integer. Minimum is zero
immutable DLTObsDist
    same::Bool
    distances::ClearDistances
    cdf::ReadingCDF
    std::Float64
    
    DLTObsDist(same::Bool) = new(true)
    DLTObsDist(dists::ClearDistances, cdf::ReadingCDF, std::Float64) = new(false, dists, cdf, std)
end

function rand(rng::AbstractRNG, d::DLTObsDist)
    if d.same
        return D_SAME_LOC
    end
    meas = MVector{8, Int}()
    meas[1:4] = max(0, floor(Int, d.distances.cardinal + d.std*randn(rng, 4)))
    meas[5:8] = max(0, floor(Int, d.distances.diagonal*sqrt(2) + d.std*randn(rng, 4)))
    return meas
end

function pdf(d::DLTObsDist, m::DMeas)
    if d.same
        return m == D_SAME_LOC ? 1.0 : 0.0
    elseif m == D_SAME_LOC
        return 0.0
    end
    p = 1.0
    for dir in 1:8
        if m[dir] == 0
            p *= cdf(d.cdf, dir, n_clear_cells(d.distances, dir), 0)
        else
            p *= (cdf(d.cdf, dir, n_clear_cells(d.distances, dir), m[dir]) - 
                  cdf(d.cdf, dir, n_clear_cells(d.distances, dir), m[dir]-1))
        end
    end
    return p
end

function observation(p::LaserTagPOMDP{DMeas}, sp::LTState)
    if sp.robot == sp.opponent
        return DLTObsDist(true) 
    else
        return DLTObsDist(p.dcache[sp], get(p.cdf), p.reading_std)
    end
end
