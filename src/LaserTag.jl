module LaserTag

# package code goes here
using StaticArrays
using AutoHashEquals
using Parameters
using StatsBase
using Distributions

importall POMDPs

using POMCP
import POMCP: init_N, init_V

export
    LaserTagPOMDP,
    Coord,
    LTState,
    CMeas,
    LaserTagVis,

    MoveTowards,
    MoveTowardsSampled,
    InevitableInit,

    gen_lasertag,
    tikz_pic


typealias Coord SVector{2, Int}
typealias CMeas MVector{8, Float64}
typealias DMeas MVector{8, Int}

const C_SAME_LOC = fill!(MVector{8, Float64}(), -1.0)
const D_SAME_LOC = fill!(MVector{8, Float64}(), -1)

@auto_hash_equals immutable LTState # XXX auto_hash_equals isn't correct for terminal
    robot::Coord
    opponent::Coord
    terminal::Bool
end

#=
immutable CLTObs
    same::Bool
    meas::MVector{8, Float64}
end
Base.(==)(a::CLTObs, b::CLTObs) = a.same && b.same || (!a.same && !b.same && a.meas == b.meas)
function Base.hash(o::CLTObs, h::UInt)
    if 
end
=#

immutable Floor
    n_rows::Int
    n_cols::Int
end

inside(f::Floor, c::Coord) = 0 < c[1] <= f.n_cols && 0 < c[2] <= f.n_rows

function add_if_inside(f::Floor, x::Coord, dx::Coord)
    if inside(f, x + dx)
        return x + dx
    else
        return x
    end
end


include("distance_cache.jl")

@with_kw immutable LaserTagPOMDP <: POMDP{LTState, Int, CMeas}
    tag_reward::Float64     = 10.0
    step_cost::Float64      = 1.0
    discount::Float64       = 0.95
    floor::Floor            = Floor(7, 11)
    reading_std::Float64    = 2.5
    obstacles::Set{Coord}   = Set{Coord}()
    robot_init::Coord       = Coord(1,1)
    dcache::LTDistanceCache = LTDistanceCache(floor, obstacles)
end

function gen_lasertag(n_rows::Int=7, n_cols::Int=11, n_obstacles::Int=8; rng=Base.GLOBAL_RNG, kwargs...)
    f = Floor(n_rows, n_cols)
    obs_inds = randperm(rng, n_pos(f))[1:n_obstacles] # XXX inefficient
    obs_subs = ind2sub((n_cols, n_rows), obs_inds)
    obstacles = Set{Coord}(Coord(p) for p in zip(obs_subs...))
    for c in obstacles
        if !inside(f, c)
            @show c
            @show obs_inds
            @show obs_subs
            error("not inside")
        end
    end
    r = Coord(rand(rng, 1:f.n_cols), rand(rng, 1:f.n_rows))
    return LaserTagPOMDP(;floor=f, obstacles=obstacles, robot_init=r, kwargs...)
end

opaque(p::LaserTagPOMDP, s::LTState, c::Coord) = opaque(p.floor, p.obstacles, s, c)

function opaque(floor::Floor, obstacles::Set{Coord}, s::LTState, c::Coord)
    if opaque(floor, obstacles, c)
        return true
    elseif c == s.opponent
        return true
    else
        return false
    end
end

function opaque(f::Floor, obstacles::Set{Coord}, c::Coord)
    if !inside(f, c)
        return true
    elseif c in obstacles
        return true
    else
        return false
    end
end

find_distances(p::LaserTagPOMDP, s::LTState) = find_distances(p.floor, p.obstacles, s)

include("states.jl")
include("actions.jl")
include("transition.jl")
include("observations.jl")
include("initial.jl")

function reward(p::LaserTagPOMDP, s::LTState, a::Int, sp::LTState)
    if a == TAG_ACTION
        if s.robot == s.opponent
            @assert sp.terminal
            return p.tag_reward
        else
            return -p.tag_reward
        end
    else
        return -p.step_cost
    end
end

isterminal(p::LaserTagPOMDP, s::LTState) = s.terminal
discount(p::LaserTagPOMDP) = p.discount

include("heuristics.jl")
include("visualization.jl")


#=
generate_s(p::LaserTagPOMDP, s::LTState, a::Int, rng::AbstractRNG) = first(generate_sr(p, s, a, rng))

function generate_sr(p::LaserTagPOMDP, s::LTState, a::Int, rng::AbstractRNG)
    terminal = false
    if a == TAG_ACTION
        if s.robot == s.opponent
            reward = p.tag_reward
            terminal = true
        else
            reward = -p.tag_reward
        end
    else
        reward = -p.step_cost
    end

    opp = s.opponent
    rob = s.robot

    # opponent behavior (see base_tag.cpp line 576)
    opp_r = rand(rng)
    if opp_r < 0.4 # 0.4 chance of moving in x direction
        if opp[1] == rob[1]
            if opp_r < 0.2
                opp_next = add_if_inside(p.floor, opp, Coord(1, 0))
            else
                opp_next = add_if_inside(p.floor, opp, Coord(-1, 0))
            end
        else
            dx = opp[1] > rob[1] ? 1 : -1
            opp_next = add_if_inside(p.floor, opp, Coord(dx, 0))
        end
    elseif opp_r < 0.8 # 0.4 chance of moving in y direction
        if opp[2] == rob[2]
            if opp_r < 0.6
                opp_next = add_if_inside(p.floor, opp, Coord(0, 1))
            else
                opp_next = add_if_inside(p.floor, opp, Coord(0, -1))
            end
        else
            dy = opp[2] > rob[2] ? 1 : -1
            opp_next = add_if_inside(p.floor, opp, Coord(0, dy))
        end
    else # 0.2 chance staying the same
        opp_next = opp
    end

    rob = add_if_inside(p.floor, rob, ACTION_DIRS[a])

    return LTState(rob, opp_next, terminal), reward
end
=#

end # module
