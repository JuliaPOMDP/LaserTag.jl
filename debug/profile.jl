using LaserTag
using Base.Test
using POMDPToolbox
using ParticleFilters
using ProfileView

p = gen_lasertag(3,4,2)

pol = RandomPolicy(p, rng=MersenneTwister(1))

sim = RolloutSimulator(max_steps=100, rng=MersenneTwister(2))

filter = SIRParticleFilter(p, 10000)

@time simulate(sim, p, pol, filter)
@time simulate(sim, p, pol, filter)
simulate(sim, p, pol, filter)

Profile.clear()
@profile simulate(sim, p, pol, filter)

ProfileView.view()
