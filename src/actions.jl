n_actions(::Type{LaserTagPOMDP}) = 9
n_actions(::LaserTagPOMDP) = 9
actions(p::LaserTagPOMDP) = 1:n_actions(p)

const ACTION_NAMES = SVector("north",
                             "south",
                             "east",
                             "west",
                             "northeast",
                             "southeast",
                             "southwest",
                             "northwest",
                             "tag")

const ACTION_DIRS = SVector(Coord(0,1),
                            Coord(0,-1),
                            Coord(1,0),
                            Coord(-1,0),
                            Coord(1,1),
                            Coord(1,-1),
                            Coord(-1,-1),
                            Coord(-1,1),
                            Coord(0,0))

const CARDINALS = SVector(Coord(0,1),
                            Coord(0,-1),
                            Coord(1,0),
                            Coord(-1,0))

const DIAGONALS = SVector(Coord(1,1),
                         Coord(1,-1),
                         Coord(-1,-1),
                         Coord(-1,1))

const DIR_TO_ACTION = Dict{Coord, Int}(c=>i for (i,c) in enumerate(ACTION_DIRS))
