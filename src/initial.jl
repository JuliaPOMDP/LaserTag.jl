immutable LTInitialBelief
    robot_init::Coord
    floor::Floor
end

Base.eltype(::Type{LTInitialBelief}) = LTState
iterator(b::LTInitialBelief) = collect(LTState(b.robot_init, Coord(x,y), false) for x in 1:b.floor.n_cols, y in 1:b.floor.n_rows)

function rand(rng::AbstractRNG, b::LTInitialBelief)
    opp = SVector(rand(rng, 1:b.floor.n_cols), rand(rng, 1:b.floor.n_rows))
    return LTState(b.robot_init, opp, false)
end

function pdf(b::LTInitialBelief, s::LTState)
    if !s.terminal && s.robot == b.robot_init
        return 1/n_pos(b.floor)
    else
        return 0.0
    end
end

initial_state_distribution(p::LaserTagPOMDP) = LTInitialBelief(p.robot_init, p.floor)
