#=
type LTDistanceCache
    distances::Nullable{Vector{SVector{8, Int}}}
end
LTDistanceCache() = LTDistanceCache(nothing)

function fill_cache!(c::LTDistanceCache, p::LaserTagPOMDP)
    dists = Array(SVector{8, Int}, n_states(p))
    for iterator(states(p))
        
    end
end

function getindex(c::LTDistanceCache, s::LTState)
    get(c.distances)
end
=#

immutable ClearDistances
    cardinal::SVector{4, Int}
    diagonal::SVector{4, Int}
end

function find_distances(p::LaserTagPOMDP, s::LTState)
    card = MVector{4, Int}()
    diag = MVector{4, Int}()
    for i in 1:4
        d = 1
        while !opaque(p, s, s.robot+(d)*CARDINALS[i])
            d += 1
        end
        card[i] = d-1
    end
    for i in 1:4
        d = 1
        while !opaque(p, s, s.robot+(d)*DIAGONALS[i])
            d += 1
        end
        diag[i] = d-1
    end
    return ClearDistances(card, diag)
end

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
    nd = Normal(0.0, d.std)
    diff = MVector{8, Float64}()
    diff[1:4] = m[1:4] - d.distances.cardinal
    diff[5:8] = m[5:8] - d.distances.diagonal*sqrt(2)
    return prod(pdf(nd, diff))
end

function observation(p::LaserTagPOMDP, s::LTState, a::Int, sp::LTState)
    if sp.robot == sp.opponent
        return CLTObsDist(true)
    end
    distances = find_distances(p, sp)
    return CLTObsDist(distances, p.reading_std)
end
