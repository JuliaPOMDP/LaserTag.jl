# for fully-observable case
immutable MoveTowards <: Policy end

action(p::MoveTowards, s::LTState) = DIR_TO_ACTION[sign(s.opponent-s.robot)]

immutable MoveTowardsSampled{RNG} <: Policy
    rng::RNG
end
MoveTowardsSampled() = MoveTowardsSampled(Base.GLOBAL_RNG)

function action(p::MoveTowardsSampled, b)
    s = rand(p.rng, b)
    return action(MoveTowards(), s)
end

immutable InevitableInit end 

init_N(m::InevitableInit, pomdp::LaserTagPOMDP, h::BeliefNode, a::Int) = h.node == 1 ? 0 : 1

function init_V(m::InevitableInit, pomdp::LaserTagPOMDP, h::BeliefNode, a::Int)
    # only works for POMCPOW now because of node access
    if a == TAG_ACTION
        if h.node == 1
            return 0.0
        elseif h.tree.o_labels[h.node] == C_SAME_LOC
            return pomdp.tag_reward
        else
            return -pomdp.tag_reward - pomdp.step_cost
        end
    else
        return -pomdp.step_cost
    end
end
