# for fully-observable case
immutable MoveTowards <: Policy end

function action(p::MoveTowards, s::LTState)
    # try to sneak up diagonally
    diff = s.opponent-s.robot
    dx = diff[1]
    dy = diff[2]
    if abs(dx) == 1 && abs(dy) == 1
        return DIR_TO_ACTION[SVector(dx, dy)]
    else
        if abs(dx) == 1
            dx = 0
        end
        if abs(dy) == 1
            dy = 0
        end
        return DIR_TO_ACTION[SVector(dx, dy)]
    end
end

immutable MoveTowardsSampled{RNG} <: Policy
    rng::RNG
end
MoveTowardsSampled() = MoveTowardsSampled(Base.GLOBAL_RNG)

function action(p::MoveTowardsSampled, b)
    s = rand(p.rng, b)
    return action(MoveTowards(), s)
end



immutable OptimalMLSolver <: Solver
    solver::Solver
end

immutable OptimalML{P<:Policy} <: Policy
    fo_policy::P 
end

solve(sol::OptimalMLSolver, p::Union{MDP,POMDP}) = OptimalML(solve(sol.solver, p))
action(pol::OptimalML, b) = action(pol.fo_policy, mode(b))

immutable BestExpectedSolver <: Solver
    solver::Solver
end

immutable BestExpected{P<:Policy} <: Policy
    fo_policy::P
end

solve(sol::BestExpectedSolver, p::Union{MDP,POMDP}) = BestExpected(solve(sol.solver, p))
function action(pol::BestExpected, b)
    best_eu = -Inf
    best_s = first(iterator(b))
    for s in iterator(b)
        eu = pdf(b, s)*value(pol.fo_policy, s)
        if eu > best_eu
            best_eu = eu
            best_s = s
        end
    end
    return action(pol.fo_policy, best_s)
end
