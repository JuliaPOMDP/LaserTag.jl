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

function observation(p::LaserTagPOMDP, sp::LTState)
    if sp.robot == sp.opponent
        return CLTObsDist(true)
    end
    distances = p.dcache[sp]
    return CLTObsDist(distances, p.reading_std)
end
