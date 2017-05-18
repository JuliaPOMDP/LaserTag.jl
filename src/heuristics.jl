# for fully-observable case
immutable MoveTowards <: Policy end

action(p::MoveTowards, s::LTState) = DIR_TO_ACTION[sign(s.opponent-s.robot)]
