immutable ClearDistances
    cardinal::SVector{4, Int}
    diagonal::SVector{4, Int}
end

function find_distances(f::Floor, obstacles::Set{Coord}, s::LTState)
    card = MVector{4, Int}()
    diag = MVector{4, Int}()
    for i in 1:4
        d = 1
        while !opaque(f, obstacles, s, s.robot+d*CARDINALS[i])
            d += 1
        end
        card[i] = d-1
    end
    for i in 1:4
        d = 1
        while !opaque(f, obstacles, s, s.robot+d*DIAGONALS[i])
            d += 1
        end
        diag[i] = d-1
    end
    return ClearDistances(card, diag)
end

type LTDistanceCache
    floor::Floor
    distances::Vector{ClearDistances}
end

function LTDistanceCache(f::Floor, obstacles::Set{Coord})
    dists = Array(ClearDistances, n_pos(f)^2)
    visited = falses(n_pos(f)^2)
    for i in 1:f.n_cols, j in 1:f.n_rows, k in 1:f.n_cols, l in 1:f.n_rows
        s = LTState(Coord(i,j), Coord(k,l), false)
        ii = state_index(f, s)
        visited[ii] = true
        dists[ii] = find_distances(f, obstacles, s)
    end
    @assert all(visited)
    push!(dists, ClearDistances(zeros(4), zeros(4)))
    return LTDistanceCache(f, dists)
end

Base.getindex(c::LTDistanceCache, s::LTState) = c.distances[state_index(c.floor, s)]
