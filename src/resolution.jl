# This file contains methods to solve an instance (heuristically or with CPLEX)
using CPLEX

include("generation.jl")

TOL = 0.00001


function find_first_edge(h::Array{VariableRef, 2})
    for i in 3:size(h, 1)
        for j in 3:size(h, 2)
            if JuMP.value(h[i,j]) == 1
                return (i, j)
            end
        end
    end
end

function find_path(game::Vector{Vector{Int}}, h::Array{VariableRef, 2}, v::Array{VariableRef, 2})
    path_circles = 0
    l, c = find_first_edge(h)
    previous = 1
    end_l = l
    end_c = c + 1

    path_h = Tuple{Int,Int}[]
    path_v = Tuple{Int,Int}[]
    push!(path_h, (l,c))
    if game[l-2][c-2] > 0
        path_circles += 1
    end
    while l != end_l || c != end_c
        if previous != 1 && JuMP.value(h[l,c]) == 1
            push!(path_h, (l,c))
            c += 1
            previous = 3
        elseif previous != 3 && JuMP.value(h[l,c-1]) == 1
            push!(path_h, (l,c-1))
            c -= 1
            previous = 1
        elseif previous != 2 && JuMP.value(v[l,c]) == 1
            push!(path_v, (l,c))
            l += 1
            previous = 4
        elseif previous != 4 && JuMP.value(v[l-1,c]) == 1
            push!(path_v, (l-1,c))
            l -= 1
            previous = 2
        end
        if game[l-2][c-2] > 0
            path_circles += 1
        end
    end
    return path_circles == sum(1 for i=1:size(game,1), j=1:size(game[1],1) if game[i][j]>0), path_h, path_v
end


"""
Solve an instance with CPLEX
"""
function cplexSolve(game::Vector{Vector{Int}}, to_avoid_h::Vector{Vector{Tuple{Int,Int}}}, to_avoid_v::Vector{Vector{Tuple{Int,Int}}})

    #get the size of the board
    a = size(game, 1)
    b = size(game[1], 1)

    # Create the model
    m = Model(CPLEX.Optimizer)

    set_optimizer_attribute(m, "CPXPARAM_ScreenOutput", 0)
    set_optimizer_attribute(m, "CPXPARAM_MIP_Display", 0)

    # TODO
    #println("In file resolution.jl, in method cplexSolve(), TODO: fix input and output, define the model")

    # edges between cells, 0 if not used and 1 if used
    # we add 2 cells on each side of the board an set all to 0 to simplify the constraints
    # v represents the vertical edges
    # h represents the horizontal edges
    @variable(m, v[1:(a+3), 1:(b+4)], Bin)
    @variable(m, h[1:(a+4), 1:(b+3)], Bin)

    for i in [1, 2, a+2, a+3], j in 1:b+4
        @constraint(m, v[i,j] == 0)
    end
    for i in 1:a+3, j in [1, 2, b+3, b+4]
        @constraint(m, v[i,j] == 0)
    end
    for i in [1, 2, a+3, a+4], j in 1:b+3
        @constraint(m, h[i,j] == 0)
    end
    for i in 1:a+4, j in [1, 2, b+2, b+3]
        @constraint(m, h[i,j] == 0)
    end

    # Canot be a previous solution
    for q in 1:length(to_avoid_h)
        @constraint(m, sum(h[i,j] for (i,j) in to_avoid_h[q]) 
            + sum(v[i,j] for (i,j) in to_avoid_v[q]) 
            <= length(to_avoid_h[q]) + length(to_avoid_v[q]) - 1)
    end

    # Add the constraint that enforces the black circle rules, corner, not connected directly to another corner, where a(i,j) = 2
    for i in 3:a+2, j in 3:b+2
        if game[i-2][j-2] == 2
            # corner
            @constraint(m, v[i-1,j] + v[i,j] == 1)
            @constraint(m, h[i,j-1] + h[i,j] == 1)
            # not connected directly to another corner
            @constraint(m, v[i-2,j] >= v[i-1,j])
            @constraint(m, v[i+1,j] >= v[i,j])
            @constraint(m, h[i,j-2] >= h[i,j-1])
            @constraint(m, h[i,j+1] >= h[i,j])
        end
    end

    # Add the constraint that enforces the white circle rules, straight line, connected to at least one corner, where a(i,j) = 1
    for i in 3:a+2, j in 3:b+2
        if game[i-2][j-2] == 1
            # straight line
            @constraint(m, v[i-1,j] == v[i,j])
            @constraint(m, h[i,j-1] == h[i,j])
            @constraint(m, v[i,j] + h[i,j] == 1)
            # connected to at least one corner
            @constraint(m, v[i-2,j] + v[i-1,j] + v[i,j] + v[i+1,j] <= 3)
            @constraint(m, h[i,j-2] + h[i,j-1] + h[i,j] + h[i,j+1] <= 3)
        end
    end

    # each cell/vertex has exactly 2 edges or it's not part of the solution, so 0 edges
    @variable(m, z[1:a,1:b], Bin)
    @constraint(m, [i=3:a+2, j=3:b+2], v[i-1,j] + v[i,j] + h[i,j-1] + h[i,j] == z[i-2,j-2]*2)
    
    # number of edges equals number of vertices, so have only cycles as solution
    @constraint(m, sum(v[i,j] + h[i,j] -z[i-2,j-2] for i=3:a+2, j=3:b+2) == 0)
    
    @objective(m, Max, 1)
    #@objective(m, Min, sum(v[i,j] + h[i,j] for i=3:a+2, j=3:b+2))


    # Solve the model
    optimize!(m)

    # Return:
    # 1 - true if an optimum is found
    # 2 - the resolution time
    return JuMP.primal_status(m) == MOI.FEASIBLE_POINT, h, v
    
end

function solve(game::Vector{Vector{Int}}, timer)

    # Start a chronometer
    #timer = time()

    to_avoid_h = Vector{Tuple{Int,Int}}[]
    to_avoid_v = Vector{Tuple{Int,Int}}[]

    feasible, h, v = cplexSolve(game, to_avoid_h, to_avoid_v)

    # check if solution is 1 cycle
    correct, path_h, path_v = find_path(game, h, v)

    # solve again if more than 1 cycle, until we get a solution with 1 cycle
    while !correct
        if time() - timer > 100
            feasible = false
            break
        end
        
        # For real time display in a txt file
        #UpdateSolution("realTimeCplex.txt", game, h, v)
        #sleep(3)
        
        # add the current state to not allow the same solution again
        push!(to_avoid_h, path_h)
        push!(to_avoid_v, path_v)
        
        feasible, h, v = cplexSolve(game, to_avoid_h, to_avoid_v)

        correct, path_h, path_v = find_path(game, h, v)
    end
    
    # For real time display in a txt file
    #UpdateSolution("realTimeCplex.txt", game, h, v)
    #sleep(3)

    # !!!!!!!!! Uncomment for the display of solution
    #displaySolution(game,h,v)

    return feasible, h, v
end

"""
Heuristically solve an instance
"""
"""
# For real time display in a txt file
global_game = 0
global_h = 0
global_v = 0
"""

# Faire un merge de 2 parties connexes
function merge!(connections::Vector{Vector{Tuple{Int,Int}}}, cell1::Tuple{Int,Int}, cell2::Tuple{Int,Int}, cells::Matrix{Int})
    #trouver les parties connxes des 2 cellules a connecter
    index1 = findfirst(x -> x[1] == cell1 || x[end] == cell1, connections)
    index2 = findfirst(x -> x[1] == cell2 || x[end] == cell2, connections)
    
    # For real time display in a txt file
    """
    global global_game
    global global_h
    global global_v
    UpdateTmp("realTime.txt", global_game, global_h, global_v)
    sleep(1)
    """
    # make the necessary changes before doing the merge
    # to keep connected cells as first and last in their list
    if index1 != index2
        if connections[index2][1] == cell2
            if connections[index1][1] == cell1 
                reverse!(connections[index1])
            end
            append!(connections[index1], connections[index2])
            deleteat!(connections,index2)
        else
            if connections[index1][end] == cell1 
                reverse!(connections[index1])
            end
            append!(connections[index2], connections[index1])
            deleteat!(connections,index1)
        end
    else 
        if connections[index1][1] == cell1
            push!(connections[index1], cell1)
        else
            push!(connections[index1], cell2)
        end
    end
    # update cells value
    cells[cell1[1],cell1[2]] += 1
    cells[cell2[1],cell2[2]] += 1
end

# Trouver tous ls voisins possibles d'une cellule sans cercle
function findNeighbors(game::Vector{Vector{Int}}, section::Vector{Tuple{Int,Int}}, 
        h::Matrix{Int}, v::Matrix{Int}, l::Int, c::Int, a::Int, b::Int, cells::Matrix{Int}, BFS=false)
    # Find neighbors
    neighbors = Tuple{Int,Int}[]

    # If the connected part contains all the circles it can connect to the other end of itself
    if length(section) > 1
        done = sum(1 for i=1:size(game,1), j=1:size(game[1],1) if game[i][j]>0) == sum(1 for (i,j) in section if game[i][j]>0)
    else
        done = false
    end
    
    # Up
    if l > 1 && v[l-1,c] == 0 && cells[l-1,c] < 2 && 
            (game[l-1][c] != 2 || v[l-2,c] == 0) && 
            ( done || (section[1] != (l-1,c) && section[length(section)] != (l-1,c)))
        push!(neighbors, (l-1,c))
    end
    # Down
    if l < a && v[l,c] == 0 && cells[l+1,c] < 2 && 
            (game[l+1][c] != 2 || v[l+1,c] == 0) && 
            ( done || (section[1] != (l+1,c) && section[length(section)] != (l+1,c)))
        push!(neighbors, (l+1,c))
    end
    # Left
    if c > 1 && h[l,c-1] == 0 && cells[l,c-1] < 2 && 
            (game[l][c-1] != 2 || h[l,c-2] == 0) && 
            ( done || (section[1] != (l,c-1) && section[length(section)] != (l,c-1)))
        push!(neighbors, (l,c-1))
    end
    #Right
    if c < b && h[l,c] == 0 && cells[l,c+1] < 2 && 
            (game[l][c+1] != 2 || h[l,c+1] == 0) && 
            ( done || (section[1] != (l,c+1) && section[length(section)] != (l,c+1)))
        push!(neighbors, (l,c+1))
    end

    # Only when no other move is possible, 
    # use a breadth-first search on a neighbor when 2 are available, 
    # while keeping track of visited cells to not revisit in cycles,
    # if none is connected to a cell with cells = 1 then path of this neighbor is block,
    # we take the other neighbor
    if BFS
        for neighbor in neighbors
            if cells[neighbor[1],neighbor[2]] == 0 && game[neighbor[1]][neighbor[2]] == 0
                final_neighbor = (neighbor[1],neighbor[2])
                visited = Tuple{Int,Int}[]
                push!(visited, (l,c))
                queue = Tuple{Int,Int}[]
                push!(queue, neighbor)
                possible = 0
                while !isempty(queue)
                    node = popfirst!(queue)
                    if !(node in visited)
                        push!(visited, node)
                        for new_neighbor in findNeighbors(game, [(0,0)], h, v, node[1], node[2], a, b, cells)
                            if !(new_neighbor in visited)
                                push!(queue, new_neighbor)
                                # If a possible neighbor is found
                                if cells[new_neighbor[1],new_neighbor[2]] == 1
                                    final_neighbor = (new_neighbor[1],new_neighbor[2])
                                    possible += 1
                                    if possible > 1
                                        break
                                    end
                                end
                            end
                        end
                        if possible > 1
                            break
                        end
                    end
                end
                i = findfirst(x -> x == neighbor, neighbors)
                # If no possible neighbors, this path is blocked
                if possible == 0
                    deleteat!(neighbors,i)
                # If only one possible neighbor not from same connected part, garanteed connection
                elseif possible == 1 && !(final_neighbor in neighbors)
                    neighbors = neighbors[i:i]
                    break
                elseif possible == 1
                    neighbors = [final_neighbor]
                    break
                end
            end
        end
    end
    return neighbors
end

function heuristicSolve(game::Vector{Vector{Int}})
    timer = time()
    # TODO
    #println("In file resolution.jl, in method heuristicSolve(), TODO: fix input and output, define the model")
    
    
    a = size(game, 1)
    b = size(game[1], 1)

    # Horizontal and vertical connections
    h = zeros(Int, a, b-1)
    v = zeros(Int, a-1, b)
    
    """
    # For real time display in a txt file
    global global_game
    global_game = game
    global global_h
    global_h = h
    global global_v
    global_v = v
    """
    # Number of connections of each cell
    cells = zeros(Int, a, b)

    # Connected parts
    connections = Vector{Tuple{Int,Int}}[]
    for i in 1:a, j in 1:b
        push!(connections,[(i,j)])
    end

    # first and last line
    for i in 1:b
        # black dot first line
        if game[1][i] == 2
            if v[1,i] == 0
                v[1,i] = 1
                merge!(connections,(1,i),(2,i),cells)
            end
            if v[2,i] == 0
                v[2,i] = 1
                merge!(connections,(2,i),(3,i),cells)
            end
        # white dot first line
        elseif game[1][i] == 1
            if h[1,i] == 0
                h[1,i] = 1
                merge!(connections,(1,i),(1,i+1),cells)
            end
            if h[1,i-1] == 0
                h[1,i-1] = 1
                merge!(connections,(1,i),(1,i-1),cells)
            end
        end
        # black dot last line
        if game[a][i] == 2
            if v[a-1,i] == 0
                v[a-1,i] = 1
                merge!(connections,(a,i),(a-1,i),cells)
            end
            if v[a-2,i] == 0
                v[a-2,i] = 1
                merge!(connections,(a-1,i),(a-2,i),cells)
            end
        # white dot last line
        elseif game[a][i] == 1
            if h[a,i] == 0
                h[a,i] = 1
                merge!(connections,(a,i),(a,i+1),cells)
            end
            if h[a,i-1] == 0
                h[a,i-1] = 1
                merge!(connections,(a,i),(a,i-1),cells)
            end
        end
    end
    # first and last column
    for i in 1:a
        # black dot first column
        if game[i][1] == 2
            if h[i,1] == 0
                h[i,1] = 1
                merge!(connections,(i,1),(i,2),cells)
            end
            if h[i,2] == 0
                h[i,2] = 1
                merge!(connections,(i,2),(i,3),cells)
            end
        # white dot first column
        elseif game[i][1] == 1
            if v[i,1] == 0
                v[i,1] = 1
                merge!(connections,(i,1),(i+1,1),cells)
            end
            if v[i-1,1] == 0
                v[i-1,1] = 1
                merge!(connections,(i,1),(i-1,1),cells)
            end
        end
        # black dot last column
        if game[i][b] == 2
            if h[i,b-1] == 0
                h[i,b-1] = 1
                merge!(connections,(i,b),(i,b-1),cells)
            end
            if h[i,b-2] == 0
                h[i,b-2] = 1
                merge!(connections,(i,b-1),(i,b-2),cells)
            end
        # white dot last column
        elseif game[i][b] == 1
            if v[i,b] == 0
                v[i,b] = 1
                merge!(connections,(i,b),(i+1,b),cells)
            end
            if v[i-1,b] == 0
                v[i-1,b] = 1
                merge!(connections,(i,b),(i-1,b),cells)
            end
        end
    end

    # second and pre-last line
    for i in 1:b
        # black dot second line
        if game[2][i] == 2
            if v[2,i] == 0
                v[2,i] = 1
                merge!(connections,(2,i),(3,i),cells)
            end
            if v[3,i] == 0
                v[3,i] = 1
                merge!(connections,(3,i),(4,i),cells)
            end
        end
        # black dot pre-last line
        if game[a-1][i] == 2
            if v[a-2,i] == 0
                v[a-2,i] = 1
                merge!(connections,(a-1,i),(a-2,i),cells)
            end
            if v[a-3,i] == 0
                v[a-3,i] = 1
                merge!(connections,(a-2,i),(a-3,i),cells)
            end
        end
    end
    # second and pre-last column
    for i in 1:a
        # black dot scond column
        if game[i][2] == 2
            if h[i,2] == 0
                h[i,2] = 1
                merge!(connections,(i,2),(i,3),cells)
            end
            if h[i,3] == 0
                h[i,3] = 1
                merge!(connections,(i,3),(i,4),cells)
            end
        end
        # black dot pre-last column
        if game[i][b-1] == 2
            if h[i,b-2] == 0
                h[i,b-2] = 1
                merge!(connections,(i,b-1),(i,b-2),cells)
            end
            if h[i,b-3] == 0
                h[i,b-3] = 1
                merge!(connections,(i,b-2),(i,b-3),cells)
            end
        end
    end
    solved = false

    # Perform guaranteed actions, should never return false unless there is no possible solution
    if !loop_grid!(game, connections, h, v, a, b, cells)
        return false
    end

    # Check if a solution was found, if one of the connected paths is a cycle containing all circles
    for section in connections
        if length(section) > 1
            if sum(1 for i=1:size(game,1), j=1:size(game[1],1) if game[i][j]>0) == sum(1 for (i,j) in section if game[i][j]>0) &&
                    section[1] == section[end]
                solved = true
                break
            else
                break
            end
        end
    end

    # If we still don't have a solution we start doing trial and error
    # starting with empty white cercles since they have 2 options
    # then black circles connected one, also 2 options
    # then empty black circles, 4 options
    if !solved
        # save a checkpoint to start a trial and error section where we try a new line and see how it goes
        checkpoint_connections = [deepcopy(connections)]
        checkpoint_h = [copy(h)]
        checkpoint_v = [copy(v)]
        checkpoint_cells = [copy(cells)]

        # recursively try steps starting with the best steps first
        solved = trial_and_error!(game, connections, h, v, a, b, cells, checkpoint_connections, checkpoint_h, checkpoint_v, checkpoint_cells, timer)
    end

    # Check if a solution was found, if one of the connected paths is a cycle containing all circles
    for section in connections
        if length(section) > 1
            if sum(1 for i=1:size(game,1), j=1:size(game[1],1) if game[i][j]>0) == sum(1 for (i,j) in section if game[i][j]>0) &&
                    section[1] == section[end]
                solved = true
                break
            else
                break
            end
        end
    end

    # Check if all constraints on circles are correct, therefore solution is correct
    for l=1:a, c=1:b
        # Check for black circle
        if game[l][c] == 2
            if !((l < a-1 && c < b-1 && h[l,c] + h[l,c+1] + v[l,c] + v[l+1,c] == 4) ||
                    (l < a-1 && c > 2 && h[l,c-1] + h[l,c-2] + v[l,c] + v[l+1,c] == 4) ||
                    (l > 2 && c < b-1 && h[l,c] + h[l,c+1] + v[l-1,c] + v[l-2,c] == 4) ||
                    (l > 2 && c > 2 && h[l,c-1] + h[l,c-2] + v[l-1,c] + v[l-2,c] == 4))
                solved = false
            end
        # Check for white circle
        elseif game[l][c] == 1
            if !(c > 1 && c < b && h[l,c-1] + h[l,c] == 2 && !(c > 2 && c < b-1 && h[l,c-2] + h[l,c-1] + h[l,c] + h[l,c+1]  == 4)) &&
                    !(l > 1 && l < a && v[l-1,c] + v[l,c] == 2 && !(l > 2 && l < a-1 && v[l-2,c] + v[l-1,c] + v[l,c] + v[l+1,c]  == 4))
                solved = false
            end
        end
    end
    
    # !!!!!!!!! Uncomment for the display of solution
    #displayTmp(game,h,v)

    return solved, h, v

end

# Trial and error, we look for an empty white circle, if none left then we check cells that have one connection
# for white circles e try a line and see how it goes, if things don't work out we use the other line
# for other cells that have one connection we try the neighbors one by one
# This function works recursively to try lines and remove them when it fails, until we reach a solution
function trial_and_error!(game::Vector{Vector{Int}}, connections::Vector{Vector{Tuple{Int,Int}}}, h::Matrix{Int}, v::Matrix{Int}, 
        a::Int, b::Int, cells::Matrix{Int}, checkpoint_connections::Vector{Vector{Vector{Tuple{Int,Int}}}}, 
        checkpoint_h::Vector{Matrix{Int}}, checkpoint_v::Vector{Matrix{Int}},checkpoint_cells::Vector{Matrix{Int}}, timer::Float64)
    # Ran out of time
    if time() - timer > 100
        pop!(checkpoint_connections)
        pop!(checkpoint_h)
        pop!(checkpoint_v)
        pop!(checkpoint_cells)
        return false
    end

    solved = false
    # finds the next cell to consider, either white circle or a black circle with 1 connection, 
    # or a black circle without connections, or a cell with 1 connection
    # If none found return false, no further actions possible
    circle = findCircle(game, a, b, cells)
    if circle == (0,0)
        pop!(checkpoint_connections)
        pop!(checkpoint_h)
        pop!(checkpoint_v)
        pop!(checkpoint_cells)
        return false
    end
    
    l = circle[1]
    c = circle[2]

    # if white circle
    if game[l][c] == 1
        h[l,c] = 1
        merge!(connections,(l,c),(l,c+1),cells)
        h[l,c-1] = 1
        merge!(connections,(l,c),(l,c-1),cells)
    # if black circle
    elseif game[l][c] == 2
        if h[l,c] == 1 || h[l,c-1] == 1
            if v[l,c] == 0
                v[l,c] = 1
                merge!(connections,(l,c),(l+1,c),cells)
            end
            if v[l+1,c] == 0
                v[l+1,c] = 1
                merge!(connections,(l+1,c),(l+2,c),cells)
            end
        else
            if h[l,c] == 0
                h[l,c] = 1
                merge!(connections,(l,c),(l,c+1),cells)
            end
            if h[l,c+1] == 0
                h[l,c+1] = 1
                merge!(connections,(l,c+1),(l,c+2),cells)
            end
        end
    # if cell with 1 connection
    else
        section = connections[findfirst(x -> x[1] == (l,c) || x[end] == (l,c), connections)]
       
        # If no neighbors then false position, go back
        neighbors = findNeighbors(game, section, h, v, l, c, a, b, cells)
        if length(neighbors) == 0
            empty!(connections)
            for section in checkpoint_connections[length(checkpoint_connections)]
                push!(connections, copy(section))
            end
            pop!(checkpoint_connections)
            h .= checkpoint_h[length(checkpoint_h)]
            pop!(checkpoint_h)
            v .= checkpoint_v[length(checkpoint_v)]
            pop!(checkpoint_v)
            cells .= checkpoint_cells[length(checkpoint_cells)]
            pop!(checkpoint_cells)
            return false
        end

        # Start with first neighbor
        neighbor = neighbors[1]
        if neighbor[1] > l
            v[l,c] = 1
            merge!(connections,(l,c),(l+1,c),cells)
        elseif neighbor[1] < l
            v[l-1,c] = 1
            merge!(connections,(l,c),(l-1,c),cells)
        elseif neighbor[2] > c
            h[l,c] = 1
            merge!(connections,(l,c),(l,c+1),cells)
        elseif neighbor[2] < c
            h[l,c-1] = 1
            merge!(connections,(l,c),(l,c-1),cells)
        end
    end

    wrong = false

    # perform garanteed actions based on the added lines
    if !loop_grid!(game, connections, h, v, a, b, cells)
        wrong = true
    end

    # check if it became unsolvable
    for l=1:a, c=1:b
        # Check for black circle
        if game[l][c] == 2 && cells[l,c] == 2
            if !((l < a-1 && c < b-1 && h[l,c] + h[l,c+1] + v[l,c] + v[l+1,c] == 4) ||
                    (l < a-1 && c > 2 && h[l,c-1] + h[l,c-2] + v[l,c] + v[l+1,c] == 4) ||
                    (l > 2 && c < b-1 && h[l,c] + h[l,c+1] + v[l-1,c] + v[l-2,c] == 4) ||
                    (l > 2 && c > 2 && h[l,c-1] + h[l,c-2] + v[l-1,c] + v[l-2,c] == 4))
                wrong = true
                break
            end
        # Check for white circle
        elseif game[l][c] == 1 && cells[l,c] == 2
            if !(c > 1 && c < b && h[l,c-1] + h[l,c] == 2 && !(c > 2 && c < b-1 && h[l,c-2] + h[l,c-1] + h[l,c] + h[l,c+1]  == 4)) &&
                    !(l > 1 && l < a && v[l-1,c] + v[l,c] == 2 && !(l > 2 && l < a-1 && v[l-2,c] + v[l-1,c] + v[l,c] + v[l+1,c]  == 4))
                wrong = true
                break
            end
        end
    end
    # check for blocked paths
    if !wrong
        for section in connections
            for coords in (1,length(section))
                i = section[coords][1]
                j = section[coords][2]
                if cells[i,j] == 1 && length(findNeighbors(game, section, h, v, i, j, a, b, cells)) == 0
                    wrong = true
                    break
                end
            end
            if wrong
                break
            end
        end
    end
    

    # check if we got a solution, if so return true
    if !wrong
        for section in connections
            if length(section) > 1
                if sum(1 for i=1:size(game,1), j=1:size(game[1],1) if game[i][j]>0) == sum(1 for (i,j) in section if game[i][j]>0) &&
                        section[1] == section[end]
                    solved = true
                    return true
                else
                    break
                end
            end
        end
    end

    # if it's not unsolvable, we can continue
    if !wrong
        # make a new checkpoint
        push!(checkpoint_connections, deepcopy(connections))
        push!(checkpoint_h, copy(h))
        push!(checkpoint_v, copy(v))
        push!(checkpoint_cells, copy(cells))

        # recursively continue the trial and error, if no solution found at the end then this step is wrong
        wrong = !trial_and_error!(game, connections, h, v, a, b, cells, checkpoint_connections, checkpoint_h, checkpoint_v, checkpoint_cells, timer)
    end
    # if this step was wrong
    if wrong
        # revert back to the last checkpoint
        empty!(connections)
        for section in checkpoint_connections[length(checkpoint_connections)]
            push!(connections, copy(section))
        end
        h .= checkpoint_h[length(checkpoint_h)]
        v .= checkpoint_v[length(checkpoint_v)]
        cells .= checkpoint_cells[length(checkpoint_cells)]

        # try the other option
        # if white circle
        if game[l][c] == 1
            v[l,c] = 1
            merge!(connections,(l,c),(l+1,c),cells)
            v[l-1,c] = 1
            merge!(connections,(l,c),(l-1,c),cells)
            # if black circle
        elseif game[l][c] == 2
            if h[l,c] == 1 || h[l,c-1] == 1
                if v[l-1,c] == 0
                    v[l-1,c] = 1
                    merge!(connections,(l,c),(l-1,c),cells)
                end
                if v[l-2,c] == 0
                    v[l-2,c] = 1
                    merge!(connections,(l-1,c),(l-2,c),cells)
                end
            else
                if h[l,c-1] == 0
                    h[l,c-1] = 1
                    merge!(connections,(l,c),(l,c-1),cells)
                end
                if h[l,c-2] == 0
                    h[l,c-2] = 1
                    merge!(connections,(l,c-1),(l,c-2),cells)
                end
            end
        # if cell with 1 connection
        else
            neighbor = neighbors[end]
            if neighbor[1] > l
                v[l,c] = 1
                merge!(connections,(l,c),(l+1,c),cells)
            elseif neighbor[1] < l
                v[l-1,c] = 1
                merge!(connections,(l,c),(l-1,c),cells)
            elseif neighbor[2] > c
                h[l,c] = 1
                merge!(connections,(l,c),(l,c+1),cells)
            elseif neighbor[2] < c
                h[l,c-1] = 1
                merge!(connections,(l,c),(l,c-1),cells)
            end
        end

        # perform garanteed actions based on the added lines, if bad position go back to checkpoint
        if !loop_grid!(game, connections, h, v, a, b, cells)
            empty!(connections)
            for section in checkpoint_connections[length(checkpoint_connections)]
                push!(connections, copy(section))
            end
            pop!(checkpoint_connections)
            h .= checkpoint_h[length(checkpoint_h)]
            pop!(checkpoint_h)
            v .= checkpoint_v[length(checkpoint_v)]
            pop!(checkpoint_v)
            cells .= checkpoint_cells[length(checkpoint_cells)]
            pop!(checkpoint_cells)

            return false
        end

        wrong = false
        # check if it became unsolvable
        for l=1:a, c=1:b
            # Check for black circle
            if game[l][c] == 2 && cells[l,c] == 2
                if !((l < a-1 && c < b-1 && h[l,c] + h[l,c+1] + v[l,c] + v[l+1,c] == 4) ||
                        (l < a-1 && c > 2 && h[l,c-1] + h[l,c-2] + v[l,c] + v[l+1,c] == 4) ||
                        (l > 2 && c < b-1 && h[l,c] + h[l,c+1] + v[l-1,c] + v[l-2,c] == 4) ||
                        (l > 2 && c > 2 && h[l,c-1] + h[l,c-2] + v[l-1,c] + v[l-2,c] == 4))
                    wrong = true
                    break
                end
            # Check for white circle
            elseif game[l][c] == 1 && cells[l,c] == 2
                if !(c > 1 && c < b && h[l,c-1] + h[l,c] == 2 && !(c > 2 && c < b-1 && h[l,c-2] + h[l,c-1] + h[l,c] + h[l,c+1]  == 4)) &&
                        !(l > 1 && l < a && v[l-1,c] + v[l,c] == 2 && !(l > 2 && l < a-1 && v[l-2,c] + v[l-1,c] + v[l,c] + v[l+1,c]  == 4))
                    wrong = true
                    break
                end
            end
        end

        # check for blocked paths
        if !wrong
            for section in connections
                for coords in (1,length(section))
                    i = section[coords][1]
                    j = section[coords][2]
                    if cells[i,j] == 1 && length(findNeighbors(game, section, h, v, i, j, a, b, cells)) == 0
                        wrong = true
                        break
                    end
                end
                if wrong
                    break
                end
            end
        end

        # check if we got a solution, if so return true
        if !wrong
            for section in connections
                if length(section) > 1
                    if sum(1 for i=1:size(game,1), j=1:size(game[1],1) if game[i][j]>0) == sum(1 for (i,j) in section if game[i][j]>0) &&
                            section[1] == section[end]
                        solved = true
                        return true
                    else
                        break
                    end
                end
            end
        end

        # if it's not unsolvable, we can continue
        if !wrong
            push!(checkpoint_connections, deepcopy(connections))
            push!(checkpoint_h, copy(h))
            push!(checkpoint_v, copy(v))
            push!(checkpoint_cells, copy(cells))

            # recursively continue the trial and error, if no solution found at the end then this step is wrong
            wrong = !trial_and_error!(game, connections, h, v, a, b, cells, checkpoint_connections, checkpoint_h, checkpoint_v, checkpoint_cells, timer)
        end
        # if this step was wrong
        if wrong

            # revert back to last checkpoint and delete it to go back a move as this one failed, we return false
            empty!(connections)
            for section in checkpoint_connections[length(checkpoint_connections)]
                push!(connections, copy(section))
            end
            pop!(checkpoint_connections)
            h .= checkpoint_h[length(checkpoint_h)]
            pop!(checkpoint_h)
            v .= checkpoint_v[length(checkpoint_v)]
            pop!(checkpoint_v)
            cells .= checkpoint_cells[length(checkpoint_cells)]
            pop!(checkpoint_cells)
            
            return false
        end
    end

    # if the recursion did not fail then we found a solution, so we return true
    return true
end


# Find the next cell to use for trial and error part, by the order given previously
# white circle -> black circle with 1 connection -> black circle without connections -> no circle with 1 connection
function findCircle(game::Vector{Vector{Int}}, a::Int, b::Int, cells::Matrix{Int})
    for i=1:a, j=1:b
        if cells[i,j] == 0 && game[i][j] == 1
            return (i,j)
        end
    end
    for i=1:a, j=1:b
        if cells[i,j] == 1 && game[i][j] == 2
            return (i,j)
        end
    end
    for i=1:a, j=1:b
        if cells[i,j] == 0 && game[i][j] == 2
            return (i,j)
        end
    end
    for i=1:a, j=1:b
        if cells[i,j] == 1 && game[i][j] == 0
            return (i,j)
        end
    end
    return (0,0)
end


# Loop through the grid and perform all the garanteed actions until none left
function loop_grid!(game::Vector{Vector{Int}}, connections::Vector{Vector{Tuple{Int,Int}}}, 
        h::Matrix{Int}, v::Matrix{Int}, a::Int, b::Int, cells::Matrix{Int})
    change = true
    check_path = false # for long algorithms to only apply when no more moves found, It's for BFS
    
    while change
        change = false
        for section in connections
            for coords in (1,length(section))
                l = section[coords][1]
                c = section[coords][2]

                # If white circle
                if game[l][c] == 1

                    # Connected on one side, add the opposite connection
                    if cells[l,c] == 1
                        if h[l,c] == 1
                            if cells[l,c-1] == 2 
                                return false
                            end
                            h[l,c-1] = 1
                            merge!(connections,(l,c),(l,c-1),cells)
                        elseif h[l,c-1] == 1
                            if cells[l,c+1] == 2 
                                return false
                            end
                            h[l,c] = 1
                            merge!(connections,(l,c),(l,c+1),cells)
                        elseif v[l,c] == 1
                            if cells[l-1,c] == 2 
                                return false
                            end
                            v[l-1,c] = 1
                            merge!(connections,(l,c),(l-1,c),cells)
                        elseif v[l-1,c] == 1
                            if cells[l+1,c] == 2 
                                return false
                            end
                            v[l,c] = 1
                            merge!(connections,(l,c),(l+1,c),cells)
                        end
                        change = true
                    
                    # Not connected, check if one direction is blocked 
                    # Or if 2 white circles adjacent on opposite sides
                    # Or 1 white circle and a parallal line adjacent on opposite sides
                    # Or if next to a black circle connected oppositely
                    # Or if one line would cause a cycle
                    # and connect accordingly
                    elseif cells[l,c] == 0
                        index1 = findfirst(x -> x[1] == (l+1,c) || x[end] == (l+1,c), connections)
                        index2 = findfirst(x -> x[1] == (l-1,c) || x[end] == (l-1,c), connections)
                        index3 = findfirst(x -> x[1] == (l,c+1) || x[end] == (l,c+1), connections)
                        index4 = findfirst(x -> x[1] == (l,c-1) || x[end] == (l,c-1), connections)
                        # Check up and down
                        if cells[l-1,c] == 2 || cells[l+1,c] == 2 || v[l,c] == -1 || v[l-1,c] == -1 ||
                                (game[l-1][c] == 1 && game[l+1][c] == 1) || 
                                (l > 2 && l < a-1 && ((game[l-1][c] == 1 && v[l+1,c] == 1) || 
                                (game[l+1][c] == 1 && v[l-2,c] == 1) || (v[l+1,c] == 1 && v[l-2,c] == 1))) ||
                                (game[l-1][c] == 2 && v[l-2,c] == 1) || (game[l+1][c] == 2 && v[l+1,c] == 1) ||
                                index1 == index2
                            if cells[l,c+1] == 2 || cells[l,c-1] == 2
                                return false
                            end
                            h[l,c] = 1
                            merge!(connections,(l,c),(l,c+1),cells)
                            h[l,c-1] = 1
                            merge!(connections,(l,c),(l,c-1),cells)
                            change = true
                        # Check left and right
                        elseif cells[l,c-1] == 2 || cells[l,c+1] == 2 || h[l,c] == -1 || h[l,c-1] == -1 ||
                                (game[l][c-1] == 1 && game[l][c+1] == 1) ||
                                (c > 2 && c < b-1 && ((game[l][c-1] == 1 && h[l,c+1] == 1) || 
                                (game[l][c+1] == 1 && h[l,c-2] == 1) || (h[l,c+1] == 1 && h[l,c-2] == 1))) ||
                                (game[l][c-1] == 2 && h[l,c-2] == 1) || (game[l][c+1] == 2 && h[l,c+1] == 1) ||
                                index3 == index4
                            if cells[l+1,c] == 2 || cells[l-1,c] == 2
                                return false
                            end
                            v[l,c] = 1
                            merge!(connections,(l,c),(l+1,c),cells)
                            v[l-1,c] = 1
                            merge!(connections,(l,c),(l-1,c),cells)
                            change = true
                        end
                    end

                # If black circle
                elseif game[l][c] == 2
                    # Check if adjacent black circle,
                    # Or if an adjacent cell has 1 connection blocking a 2 long line,
                    # then it has a line on opposite side

                    # Check up
                    if l > 2 && l < a-1
                        if v[l,c] == 0 && (v[l-1,c] != 1 || v[l-2,c] != 1) && (game[l-1][c] == 2 || 
                                (cells[l-1,c] == 2 && v[l-1,c] != 1) || 
                                (cells[l-1,c] == 1 && v[l-1,c] != 1 && v[l-2,c] != 1) || 
                                (cells[l-2,c] == 2 && v[l-2,c] != 1) ||
                                (l-2,c) == section[1] || (l-2,c) == section[length(section)] ||
                                (cells[l-2,c] != 2 && isempty(findNeighbors(game, section, h, v, l-2, c, a, b, cells))))
                            if (cells[l+1,c] == 2 && v[l,c] == 0) || (cells[l+2,c] == 2 && v[l+1,c] == 0) ||
                                    (cells[l+1,c] > 0 && v[l,c] == 0 && v[l+1,c] == 0)
                                return false
                            end
                            v[l,c] = 1
                            merge!(connections,(l,c),(l+1,c),cells)
                            change = true
                            if v[l+1,c] == 0
                                v[l+1,c] = 1
                                merge!(connections,(l+1,c),(l+2,c),cells)
                            end
                        # Check down
                        elseif v[l-1,c] == 0 && (v[l,c] != 1 || v[l+1,c] != 1) && (game[l+1][c] == 2 || 
                                (cells[l+1,c] == 2 && v[l,c] != 1) || 
                                (cells[l+1,c] == 1 && v[l,c] != 1 && v[l+1,c] != 1) ||
                                (cells[l+2,c] == 2 && v[l+1,c] != 1) ||
                                (l+2,c) == section[1] || (l+2,c) == section[length(section)] ||
                                (cells[l+2,c] != 2 && isempty(findNeighbors(game, section, h, v, l+2, c, a, b, cells))))
                            if (cells[l-1,c] == 2 && v[l-1,c] == 0) || (cells[l-2,c] == 2 && v[l-2,c] == 0) ||
                                    (cells[l-1,c] > 0 && v[l-1,c] == 0 && v[l-2,c] == 0)
                                return false
                            end
                            v[l-1,c] = 1
                            merge!(connections,(l,c),(l-1,c),cells)
                            change = true
                            if v[l-2,c] == 0
                                v[l-2,c] = 1
                                merge!(connections,(l-1,c),(l-2,c),cells)
                            end
                        end
                    end
                    # Check left
                    if c > 2 && c < b-1 && change == false
                        if h[l,c] == 0 && (h[l,c-1] != 1 || h[l,c-2] != 1) && (game[l][c-1] == 2 || 
                                (cells[l,c-1] == 2 && h[l,c-1] != 1) || 
                                (cells[l,c-1] == 1 && h[l,c-1] != 1 && h[l,c-2] != 1) ||
                                (cells[l,c-2] == 2 && h[l,c-2] != 1) ||
                                (l,c-2) == section[1] || (l,c-2) == section[length(section)] ||
                                (cells[l,c-2] != 2 && isempty(findNeighbors(game, section, h, v, l, c-2, a, b, cells))))
                            if (cells[l,c+1] == 2 && h[l,c] == 0) || (cells[l,c+2] == 2 && h[l,c+1] == 0) ||
                                    (cells[l,c+1] > 0 && h[l,c] == 0 && h[l,c+1] == 0)
                                return false
                            end
                            h[l,c] = 1
                            merge!(connections,(l,c),(l,c+1),cells)
                            change = true
                            if h[l,c+1] == 0
                                h[l,c+1] = 1
                                merge!(connections,(l,c+1),(l,c+2),cells)
                            end
                        # Check right
                        elseif h[l,c-1] == 0 && (h[l,c] != 1 || h[l,c+1] != 1) && (game[l][c+1] == 2 || 
                                (cells[l,c+1] == 2 && h[l,c] != 1) || 
                                (cells[l,c+1] == 1 && h[l,c] != 1 && h[l,c+1] != 1) ||
                                (cells[l,c+2] == 2 && h[l,c+1] != 1) ||
                                (l,c+2) == section[1] || (l,c+2) == section[length(section)] ||
                                (cells[l,c+2] != 2 && isempty(findNeighbors(game, section, h, v, l, c+2, a, b, cells))))
                            if (cells[l,c-1] == 2 && h[l,c-1] == 0) || (cells[l,c-2] == 2 && h[l,c-2] == 0) ||
                                    (cells[l,c-1] > 0 && h[l,c-1] == 0 && h[l,c-2] == 0)
                                return false
                            end
                            h[l,c-1] = 1
                            merge!(connections,(l,c),(l,c-1),cells)
                            change = true
                            if h[l,c-2] == 0
                                h[l,c-2] = 1
                                merge!(connections,(l,c-1),(l,c-2),cells)
                            end
                        end
                    end

                # If no circle but has one connection
                elseif cells[l,c] == 1
                    # If adjacnt to white
                    # Up
                    if l > 3 && l < a && v[l-1,c] == 1 && game[l-1][c] == 1 && v[l-2,c] == 1 && v[l-3,c] == 1 && v[l,c] != -1
                        v[l,c] = -1
                        change = true
                    # Down
                    elseif l > 1 && l < a-2 && v[l,c] == 1 && game[l+1][c] == 1 && v[l+1,c] == 1 && v[l+2,c] == 1 && v[l-1,c] != -1
                        v[l-1,c] = -1
                        change = true
                    # Left
                    elseif c > 3 && c < b && h[l,c-1] == 1 && game[l][c-1] == 1 && h[l,c-2] == 1 && h[l,c-3] == 1 && h[l,c] != -1
                        h[l,c] = -1
                        change = true
                    # Right
                    elseif c > 1 && c < b-2 && h[l,c] == 1 && game[l][c+1] == 1 && h[l,c+1] == 1 && h[l,c+2] == 1 && h[l,c-1] != -1
                        h[l,c-1] = -1
                        change = true
                    end

                    # Find neighbors
                    if check_path
                        # Only when no other move is possible
                        neighbors = findNeighbors(game, section, h, v, l, c, a, b, cells, true)
                    else
                        neighbors = findNeighbors(game, section, h, v, l, c, a, b, cells)
                    end

                    # If one neighbor then connect to it
                    if length(neighbors) == 1
                        nl = neighbors[1][1]
                        nc = neighbors[1][2]
                        if l != nl
                            if nl > l
                                v[l,c] = 1
                                merge!(connections,(l,c),(l+1,c),cells)
                            else
                                v[l-1,c] = 1
                                merge!(connections,(l,c),(l-1,c),cells)
                            end
                        elseif nc > c
                            h[l,c] = 1
                            merge!(connections,(l,c),(l,c+1),cells)
                        else 
                            h[l,c-1] = 1
                            merge!(connections,(l,c),(l,c-1),cells)
                        end
                        change = true
                        check_path = false
                    end

                    
                end
            end
        end

        if !change && !check_path
            change = true
            check_path = true
        end

    end
     return true
end 

"""
Solve all the instances contained in "../data" through CPLEX and heuristics

The results are written in "../res/cplex" and "../res/heuristic"

Remark: If an instance has previously been solved (either by cplex or the heuristic) it will not be solved again
"""
function solveDataSet()

    dataFolder = "../data/"
    resFolder = "../res/"

    # Array which contains the name of the resolution methods
    #resolutionMethod = ["cplex"]
    #resolutionMethod = ["heuristique"]
    resolutionMethod = ["cplex", "heuristique"]

    # Array which contains the result folder of each resolution method
    resolutionFolder = resFolder .* resolutionMethod

    # Create each result folder if it does not exist
    for folder in resolutionFolder
        if !isdir(folder)
            mkdir(folder)
        end
    end
            
    global isOptimal = false
    global solveTime = -1

    # For each instance
    # (for each file in folder dataFolder which ends by ".txt")
    for file in filter(x->occursin(".txt", x), readdir(dataFolder))  
        
        println("-- Resolution of ", file)
        game = readInputFile(dataFolder * file)

        # TODO
        #println("In file resolution.jl, in method solveDataSet(), TODO: read value returned by readInputFile()")
        
        # For each resolution method
        for methodId in 1:size(resolutionMethod, 1)
            
            outputFile = resolutionFolder[methodId] * "/" * file

            # If the instance has not already been solved by this method
            if !isfile(outputFile)
                
                fout = open(outputFile, "w")  

                resolutionTime = -1
                isOptimal = false
                
                # If the method is cplex
                if resolutionMethod[methodId] == "cplex"
                    
                    # TODO 
                    #println("In file resolution.jl, in method solveDataSet(), TODO: fix cplexSolve() arguments and returned values")
                    
                    startingTime = time()
                    # Solve it and get the results
                    isOptimal, h, v = solve(game, startingTime)
                    
                    resolutionTime = time() - startingTime
                    # If a solution is found, write it
                    if isOptimal
                        # TODO
                        #println("In file resolution.jl, in method solveDataSet(), TODO: write cplex solution in fout") 

                        displaySolution(game, h, v)
                        #writeSolution(fout, game, h, v)
                    end

                # If the method is one of the heuristics
                else

                    # Start a chronometer 
                    startingTime = time()
                    
                        
                    # TODO 
                    #println("In file resolution.jl, in method solveDataSet(), TODO: fix heuristicSolve() arguments and returned values")
                    
                    # Solve it and get the results
                    try
                        isOptimal, h, v = heuristicSolve(game)
                    catch
                    end
                    # Stop the chronometer
                    resolutionTime = time() - startingTime

                    # Write the solution (if any)
                    if isOptimal
                        displayTmp(game, h, v)
                        #writeTmp(fout, game, h, v)
                        # TODO
                        #println("In file resolution.jl, in method solveDataSet(), TODO: write the heuristic solution in fout")
                        
                    end 
                end

                println(fout, "solveTime = ", resolutionTime) 
                println(fout, "isOptimal = ", isOptimal)
                
                # TODO
                #println("In file resolution.jl, in method solveDataSet(), TODO: write the solution in fout") 
                close(fout)
            end


            # Display the results obtained with the method on the current instance
            include(outputFile)
            println(resolutionMethod[methodId], " optimal: ", isOptimal)
            println(resolutionMethod[methodId], " time: " * string(round(solveTime, sigdigits=2)) * "s\n")
        end         
    end 
end
