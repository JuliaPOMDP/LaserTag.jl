immutable LTInitialBelief
    robot_init::Coord
    floor::Floor
end

Base.eltype(::Type{LTInitialBelief}) = LTState

function rand(rng::AbstractRNG, b::LTInitialBelief)
    opp = SVector(rand(rng, 1:b.floor.n_cols), rand(rng, 1:b.floor.n_rows))
    return LTState(b.robot_init, opp, false)
end

initial_state_distribution(p::LaserTagPOMDP) = LTInitialBelief(p.robot_init, p.floor)
