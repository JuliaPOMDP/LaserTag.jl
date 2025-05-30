using LaserTag
using Random
using Test
using POMDPModels
using POMDPTools
using ParticleFilters
using POMDPs

@time p = gen_lasertag()

pol = RandomPolicy(p, rng=MersenneTwister(1))

sim = HistoryRecorder(max_steps=10, rng=MersenneTwister(2))

filter = BootstrapFilter(p, 10000)

hist = simulate(sim, p, pol, filter)

tikz_pic(LaserTagVis(p))

# discrete
p = gen_lasertag()

# check observation model consistency
rng = MersenneTwister(12)
N = 1_000_000
s = rand(rng, initialstate(p))
od = observation(p, s)
obs = [rand(rng, od) for i in 1:N]
for dir in 1:8
    counts = Dict{Int,Int}()
    for i in 1:N
        o = obs[i]
        if haskey(counts, o[dir])
            counts[o[dir]] += 1
        else
            counts[o[dir]] = 1
        end
    end
    total = 0
    for (r, count) in counts
        total += count
        if r == 0
            prob = LaserTag.cdf(od.model.cdf, dir, n_clear_cells(od.distances, dir), 0)
        else
            prob = (LaserTag.cdf(od.model.cdf, dir, n_clear_cells(od.distances, dir), r) - 
                 LaserTag.cdf(od.model.cdf, dir, n_clear_cells(od.distances, dir), r-1))
        end
        try
            @test isapprox(prob*N, count, atol=10, rtol=0.1)
        catch ex
            @show prob*N
            @show count
            rethrow(ex)
        end
    end
    @test total == N
end

# check convert_s function 
@testset "convert_s" begin
    # convert from LTState to Vector{Float64}
    s_test = rand(rng, initialstate(p))
    v_s_test = convert_s(Vector{Float64}, s_test, p)
    # convert from Vector{Float64} to LTState
    s_back = convert_s(LTState, v_s_test, p)
    @test s_back == s_test
end

pol = RandomPolicy(p, rng=MersenneTwister(1))

sim = HistoryRecorder(max_steps=10, rng=MersenneTwister(2))

filter = BootstrapFilter(p, 10000)

hist = simulate(sim, p, pol, filter)

try
    tikz_pic(LaserTagVis(p))
    render(p, first(eachstep(hist)))
    io = IOBuffer()
    show(io, MIME("image/png"), render(p, first(eachstep(hist))))
catch ex
    @warn("Unable to complete visualization tests.",
          error=sprint(showerror, ex))
end

s = rand(MersenneTwister(4), initialstate(p))
@inferred @gen(:sp, :o, :r)(p, s, 1, MersenneTwister(4))

sp, o, r = @gen(:sp, :o, :r)(p, s, 1, MersenneTwister(4))
@inferred observation(p, s, 1, sp)

show(stdout, MIME("text/plain"), LaserTagVis(cpp_emu_lasertag(4)))

include("emulate_cpp_lasertag.jl")
