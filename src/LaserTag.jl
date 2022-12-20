module LaserTag

# package code goes here
using POMDPs
using POMDPModelTools
using Random
using Printf
using StaticArrays
using SpecialFunctions
using AutoHashEquals
using POMDPSimulators
using Parameters
using StatsBase
using Distributions
using IterTools

export
    LaserTagPOMDP,
    Coord,
    LTState,
    CMeas,
    DMeas,
    LaserTagVis, MoveTowards,
    MoveTowardsSampled,
    OptimalMLSolver,
    OptimalML,
    BestExpectedSolver,
    BestExpected, DESPOTEmu, gen_lasertag,
    cpp_emu_lasertag,
    tikz_pic,
    n_clear_cells


const Coord = SVector{2,Int}
const CMeas = MVector{8,Float64}
const DMeas = MVector{8,Int}

const C_SAME_LOC = fill!(MVector{8,Float64}(undef), -1.0)
const D_SAME_LOC = fill!(MVector{8,Int64}(undef), -1)

@auto_hash_equals struct LTState # XXX auto_hash_equals isn't correct for terminal
    robot::Coord
    opponent::Coord
    terminal::Bool
end

struct Floor
    n_rows::Int
    n_cols::Int
end

inside(f::Floor, c::Coord) = 0 < c[1] <= f.n_cols && 0 < c[2] <= f.n_rows
max_diag(f::Floor) = sqrt(f.n_rows^2 + f.n_cols^2)

function add_if_inside(f::Floor, x::Coord, dx::Coord)
    if inside(f, x + dx)
        return x + dx
    else
        return x
    end
end

abstract type ObsModel end

obs_type(om::ObsModel) = obs_type(typeof(om))

include("distance_cache.jl")

@with_kw struct LaserTagPOMDP{M<:ObsModel,O<:Union{CMeas,DMeas}} <: POMDP{LTState,Int,O}
    tag_reward::Float64 = 10.0
    step_cost::Float64 = 1.0
    discount::Float64 = 0.95
    floor::Floor = Floor(7, 11)
    obstacles::Set{Coord} = Set{Coord}()
    robot_init::Union{Coord,Nothing} = nothing
    diag_actions::Bool = false
    dcache::LTDistanceCache = LTDistanceCache(floor, obstacles)
    obs_model::M = DESPOTEmu(floor, 2.5)
end

ltfloor(m::LaserTagPOMDP) = m.floor

n_cols(p::LaserTagPOMDP) = ltfloor(p).n_cols
n_rows(p::LaserTagPOMDP) = ltfloor(p).n_rows

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

function add_if_clear(f::Floor, obstacles::Set{Coord}, x::Coord, dx::Coord)
    if opaque(f, obstacles, x + dx)
        return x
    else
        return x + dx
    end
end

find_distances(p::LaserTagPOMDP, s::LTState) = find_distances(p.floor, p.obstacles, s)

include("states.jl")
include("actions.jl")
include("transition.jl")
include("obs_models.jl")
include("initial.jl")

function POMDPs.reward(p::LaserTagPOMDP, s::LTState, a::Int, sp::LTState)
    if s.terminal
        return 0.0
    end

    if a == TAG_ACTION
        if s.robot == s.opponent
            return p.tag_reward
        else
            return -p.tag_reward
        end
    else
        return -p.step_cost
    end
end


# add a function to transform a LTState to a vector 
function POMDPs.convert_s(T::Type{<:AbstractArray}, s::LTState, p::LaserTagPOMDP)
    return convert(T, vcat(s.robot, s.opponent, s.terminal))
end

# transform a vector to a LTState
function POMDPs.convert_s(T::Type{LTState}, v::AbstractArray{Float64}, p::LaserTagPOMDP)
    return LTState([v[1], v[2]], [v[3], v[4]], v[5])
end

POMDPs.isterminal(p::LaserTagPOMDP, s::LTState) = s.terminal
POMDPs.discount(p::LaserTagPOMDP) = p.discount

include("problem_gen.jl")
include("heuristics.jl")
include("visualization.jl")


end # module
