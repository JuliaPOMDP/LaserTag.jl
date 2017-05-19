using LaserTag
using Base.Test
using POMDPToolbox
using ParticleFilters
using ProfileView
using QMDP
using POMDPs

p = gen_lasertag()

solver = QMDPSolver(max_iterations=1000)
@time pol = solve(solver, p, verbose=true)

# Profile.clear()
# @profile solve(solver, p, verbose=true)
# ProfileView.view()

sim = HistoryRecorder(max_steps=5, rng=MersenneTwister(2), show_progress=true)

filter = SIRParticleFilter(p, 10000)

#=
b = initialize_belief(filter, initial_state_distribution(p))
o = rand(MersenneTwister(3), observation(p, initial_state(p, MersenneTwister(4))))
@code_warntype update(filter, b, 1, o)
=#

simulate(sim, p, pol, filter)

# @time simulate(sim, p, pol, filter)
# simulate(sim, p, pol, filter)

Profile.clear()
@profile simulate(sim, p, pol, filter)

ProfileView.view()
