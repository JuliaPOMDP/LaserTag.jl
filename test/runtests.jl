using LaserTag
using Base.Test
using POMDPToolbox
using ParticleFilters

p = LaserTagPOMDP()

pol = RandomPolicy(p, rng=MersenneTwister(1))

sim = HistoryRecorder(max_steps=10, rng=MersenneTwister(2))

filter = SIRParticleFilter(p, 10000)

hist = simulate(sim, p, pol, filter)
