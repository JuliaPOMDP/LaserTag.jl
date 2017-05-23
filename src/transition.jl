immutable LTTransDist
    terminal::Bool
    rob::Coord
    prev_opp::Coord
    probs::SVector{5, Float64} # corresponds to opp moving in the cardinal directions or staying
                     # sum(probs) is always 1!

    LTTransDist(t::Bool) = new(t)
    LTTransDist(rob::Coord, prev_opp::Coord, probs::AbstractVector) = new(false, rob, prev_opp, probs)
end

function rand(rng::AbstractRNG, d::LTTransDist)
    if d.terminal
        return LTState(d.rob, d.prev_opp, true)
    end
    i = sample(rng, Weights(d.probs, 1.0))
    if i < 5
        opp = d.prev_opp + CARDINALS[i]
    else
        opp = d.prev_opp
    end
    return LTState(d.rob, opp, false)
end

function pdf(d::LTTransDist, s::LTState)::Float64
    if d.terminal 
        return s.terminal ? 1.0 : 0.0
    elseif s.terminal || s.robot != d.rob || sumabs(s.opponent-d.prev_opp) > 1
        return 0.0
    elseif s.opponent == d.prev_opp
        return d.probs[5]
    else
        dir = s.opponent-d.prev_opp
        if dir[1] == 0
            if dir[2] == 1
                return d.probs[1]
            else
                return d.probs[2]
            end
        elseif dir[1] == 1
            return d.probs[3]
        else
            return d.probs[4]
        end
        # return d.probs[findfirst(CARDINALS, dir)]
    end
end

iterator(d::LTTransDist) = d

Base.start(d::LTTransDist) = 1
Base.done(d::LTTransDist, i::Int) = i > 5 || d.terminal && i > 1
function Base.next(d::LTTransDist, i::Int)
    if d.terminal
        return (LTState(d.rob, d.prev_opp, true), i+1)
    elseif i <= 4
        return (LTState(d.rob, d.prev_opp+CARDINALS[i], false), i+1)
    else    
        return (LTState(d.rob, d.prev_opp, false), i+1)
    end
end


function transition(p::LaserTagPOMDP, s::LTState, a::Int)
    if s.terminal || a == TAG_ACTION && s.robot == s.opponent
        return LTTransDist(true)
    end

    probs = fill!(MVector{5, Float64}(), 0.0)

    opp = s.opponent
    rob = s.robot
    f = p.floor

    # opponent behavior (see base_tag.cpp line 576)
    # 0.4 chance of moving in x direction
    if opp[1] == rob[1]
        if inside(f, opp + Coord(1,0))
            probs[3] += 0.2
        end
        if inside(f, opp + Coord(-1,0))
            probs[4] += 0.2
        end
    elseif opp[1] > rob[1] && inside(f, opp + Coord(1,0))
        probs[3] += 0.4
    elseif opp[1] < rob[1] && inside(f, opp + Coord(-1,0))
        probs[4] += 0.4
    end

    # 0.4 chance of moving in y direction
    if opp[2] == rob[2]
        if inside(f, opp + Coord(0,1))
            probs[1] += 0.2
        end
        if inside(f, opp + Coord(0,-1))
            probs[2] += 0.2
        end
    elseif opp[2] > rob[2] && inside(f, opp + Coord(0,1))
        probs[1] += 0.4
    elseif opp[2] < rob[2] && inside(f, opp + Coord(0,-1))
        probs[2] += 0.4
    end

    # 0.2 + all out of bounds mass chance staying the same
    probs[5] = 1.0 - sum(probs)

    next_rob = add_if_inside(p.floor, rob, ACTION_DIRS[a])

    return LTTransDist(next_rob, opp, probs)
end

