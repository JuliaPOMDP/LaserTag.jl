using TikzPictures
using ParticleFilters

type LaserTagVis
    p::LaserTagPOMDP
    a::Nullable{Any}
    r::Nullable{Any}
    s::Nullable{Any}
    o::Nullable{Any}
    b::Nullable{Any}
end
LaserTagVis(p; s=nothing, a=nothing, o=nothing, b=nothing, r=nothing) = LaserTagVis(p, a, r, s, o, b)
LaserTagVis(p::LaserTagPOMDP, arspobp::Tuple) = LaserTagVis(p, arspobp...)

Base.show(io::IO, mime::MIME"image/svg+xml", v::LaserTagVis) = show(io, mime, tikz_pic(v))

function Base.show(io::IO, mime::MIME"image/png", v::LaserTagVis)
    fname = tempname()
    save(PDF(fname), tikz_pic(v))
    run(`convert -flatten $(fname*".pdf") $(fname*".png")`)
    open(fname*".png") do f
        write(io, readstring(f))
    end
    run(`rm $(fname*".pdf")`)
    run(`rm $(fname*".png")`)
    return io
end

function fill_square(o::IO, x, y, color, opacity=0.5) # maybe opacity should be a keyword
    sqsize = 1.0
    println(o, "\\fill[$(color), opacity=$opacity] ($((x-1) * sqsize),$((y-1) * sqsize)) rectangle +($sqsize,$sqsize);")
end

function show_belief(o::IO, b::ParticleCollection{LTState})
    d = Dict{Coord, Int}()
    total = 0
    for p in particles(b)
        if !p.terminal
            opp = p.opponent
            if haskey(d, opp)
                d[opp] += 1
            else
                d[opp] = 1
            end
            total += 1
        end
    end
    for (opp, freq) in d
        fill_square(o, opp[1], opp[2], "yellow", sqrt(freq/total))
    end
end

function show_meas(o::IO, s::LTState, obs::Union{CMeas,DMeas})
    middle = s.robot - 0.5
    for i in 1:4
        dir = CARDINALS[i]
        start = middle+0.5*dir
        finish = start + obs[i]*dir
        draw_laser(o, start, finish)
    end
    for i in 5:8
        dir = DIAGONALS[i-4]*sqrt(2)/2
        start = middle+0.5*sqrt(2)*dir
        finish = start + obs[i]*dir 
        draw_laser(o, start, finish)
    end
end

function draw_laser(o::IO, start::AbstractVector{Float64}, finish::AbstractVector{Float64})
    println(o, "\\draw[dashed, red] $(@sprintf("(%0.3f, %0.3f)", start[1], start[2])) -- $(@sprintf("(%0.3f, %0.3f)", finish[1], finish[2]));")
end

function tikz_pic(v::LaserTagVis)
    p = v.p
    f = p.floor
    o = IOBuffer()
    sqsize=1

    println(o, "\\begin{scope}")

    println(o, "\\clip (0,0) rectangle ($(f.n_cols*sqsize),$(f.n_rows*sqsize));")

    for c in p.obstacles
        fill_square(o, c[1], c[2], "gray")
    end

    if !isnull(v.b)
        show_belief(o, get(v.b))
    end

    if !isnull(v.s)
        s = get(v.s)
        opp = s.opponent
        rob = s.robot
        fill_square(o, opp[1], opp[2], "orange")
        fill_square(o, rob[1], rob[2], "green")
        if !isnull(v.o)
            show_meas(o, s, get(v.o))
        end
        if !isnull(v.a)
            aname = ACTION_NAMES[get(v.a)]
            println(o, "\\node[above right] at ($((rob[1]-1) * sqsize), $((rob[2]-1) * sqsize)) {$aname};")
        end
        if !isnull(v.r)
            rtext = @sprintf("%0.2f", get(v.r))
            println(o, "\\node[below right] at ($((rob[1]-1) * sqsize), $((rob[2]-1) * sqsize)) {$rtext};")
        end

    end

    # # possibly for later: text in a square
    # vs = @sprintf("%0.2f", V[i])
    # println(o, "\\node[above right] at ($((xval-1) * sqsize), $((yval-1) * sqsize)) {\$$(vs)\$};")

    println(o, "\\draw[black] grid($(f.n_cols), $(f.n_rows));")

    println(o, "\\end{scope}")

    tikzDeleteIntermediate(true)
    return TikzPicture(String(take!(o)), options="scale=1.25")
end

function Base.show(io::IO, mime::MIME"text/plain", v::LaserTagVis)
    for y in n_rows(v.p):-1:1
        for x in 1:n_cols(v.p)
            printed = false
            if !isnull(v.s)
                s = get(v.s)
                if Coord(x,y) == s.robot
                    print(io, 'R')
                    printed = true
                elseif Coord(x,y) == s.opponent
                    print(io, 'O')
                    printed = true
                end
            end
            if !printed
                if Coord(x,y) in v.p.obstacles
                    print(io, '#')
                else
                    print(io, '.')
                end
            end
        end
        print(io, '\n')
    end
end
