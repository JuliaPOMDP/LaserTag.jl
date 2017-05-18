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
