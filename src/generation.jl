# This file contains methods to generate a data set of instances (i.e., sudoku grids)
include("io.jl")

"""
Generate an n*m grid

Argument
- n*m: size of the grid
"""
function generateInstance(n::Int64, m::Int64, recur = 0)

    
    # TODO
    #println("In file generation.jl, in method generateInstance(), TODO: generate an instance")
    game = Vector{Vector{Int}}(undef, n)
    for j in 1:n
        game[j] = zeros(Int, m)
    end
    h = zeros(Int, n, m-1)
    v = zeros(Int, n-1, m)
    taken = zeros(Int, n, m)

    random1 = 4
    random2 = 2
    max_recur = 150
    max_iter = n*m
    iter = 1
    empty_spots = 0.08

    for l=3:n-2, c=3:m-2
        if rand(1:1/empty_spots) ==  1 && 
                taken[l-1,c] + taken[l-1,c-1] + taken[l-1,c+1] + taken[l,c-1] + taken[l,c+1] + taken[l+1,c] + taken[l+1,c-1] + taken[l+1,c+1] == 0
            taken[l,c] = 100
        end
    end

    # Set the starting square
    l = rand(2:n-2)
    c = rand(2:m-2)
    h[l,c] = 1
    h[l-1,c] = 1
    v[l-1,c] = 1
    v[l-1,c+1] = 1
    taken[l,c] = 1
    taken[l,c+1] = 1
    taken[l-1,c] = 1
    taken[l-1,c+1] = 1

    # For real time display in a txt file
    #UpdateTmp("realTimeGeneration.txt", game, h, v)
    #sleep(0.3)
    while sum(1 for i=1:n, j=1:m if taken[i,j] == 1) * 100 / (n*m) < 90 && iter < max_iter
        iter += 1

        # expand upward or downard
        for l = 2:n-1, c = 1:m-1
            if h[l,c] == 1
                # check upward
                temp = taken[l-1,c] + taken[l-1,c+1]
                if temp == 0 && rand(1:random1) == 1
                    h[l,c] = 1 - h[l,c]
                    h[l-1,c] = 1 - h[l-1,c]
                    v[l-1,c] = 1 - v[l-1,c]
                    v[l-1,c+1] = 1 - v[l-1,c+1]
                    taken[l-1,c] = 1 - taken[l-1,c]
                    taken[l-1,c+1] = 1 - taken[l-1,c+1]
                    # For real time display in a txt file
                    #UpdateTmp("realTimeGeneration.txt", game, h, v)
                    #sleep(0.3)
                elseif temp == 1 && v[l-1,c] == 1 && rand(1:random2) == 1
                    h[l,c] = 1 - h[l,c]
                    h[l-1,c] = 1 - h[l-1,c]
                    v[l-1,c] = 1 - v[l-1,c]
                    v[l-1,c+1] = 1 - v[l-1,c+1]
                    taken[l,c] = 1 - taken[l,c]
                    taken[l-1,c+1] = 1 - taken[l-1,c+1]
                    # For real time display in a txt file
                    #UpdateTmp("realTimeGeneration.txt", game, h, v)
                    #sleep(0.3)
                elseif temp == 1 && v[l-1,c+1] == 1 && rand(1:random2) == 1
                    h[l,c] = 1 - h[l,c]
                    h[l-1,c] = 1 - h[l-1,c]
                    v[l-1,c] = 1 - v[l-1,c]
                    v[l-1,c+1] = 1 - v[l-1,c+1]
                    taken[l,c+1] = 1 - taken[l,c+1]
                    taken[l-1,c] = 1 - taken[l-1,c]
                    # For real time display in a txt file
                    #UpdateTmp("realTimeGeneration.txt", game, h, v)
                    #sleep(0.3)
                else
                    # check downward if no change upward
                    temp = taken[l+1,c] + taken[l+1,c+1]
                    if temp == 0 && rand(1:random1) == 1
                        h[l,c] = 1 - h[l,c]
                        h[l+1,c] = 1 - h[l+1,c]
                        v[l,c] = 1 - v[l,c]
                        v[l,c+1] = 1 - v[l,c+1]
                        taken[l+1,c] = 1 - taken[l+1,c]
                        taken[l+1,c+1] = 1 - taken[l+1,c+1]
                        # For real time display in a txt file
                        #UpdateTmp("realTimeGeneration.txt", game, h, v)
                        #sleep(0.3)
                    elseif temp == 1 && v[l,c] == 1 && rand(1:random2) == 1
                        h[l,c] = 1 - h[l,c]
                        h[l+1,c] = 1 - h[l+1,c]
                        v[l,c] = 1 - v[l,c]
                        v[l,c+1] = 1 - v[l,c+1]
                        taken[l,c] = 1 - taken[l,c]
                        taken[l+1,c+1] = 1 - taken[l+1,c+1]
                        # For real time display in a txt file
                        #UpdateTmp("realTimeGeneration.txt", game, h, v)
                        #sleep(0.3)
                    elseif temp == 1 && v[l,c+1] == 1 && rand(1:random2) == 1
                        h[l,c] = 1 - h[l,c]
                        h[l+1,c] = 1 - h[l+1,c]
                        v[l,c] = 1 - v[l,c]
                        v[l,c+1] = 1 - v[l,c+1]
                        taken[l,c+1] = 1 - taken[l,c+1]
                        taken[l+1,c] = 1 - taken[l+1,c]
                        # For real time display in a txt file
                        #UpdateTmp("realTimeGeneration.txt", game, h, v)
                        #sleep(0.3)
                    end
                end
            end
        end
        
        # expand left or right
        for l = 1:n-1, c = 2:m-1
            if v[l,c] == 1
                # check left
                temp = taken[l,c-1] + taken[l+1,c-1]
                if temp == 0 && rand(1:random1) == 1
                    v[l,c] = 1 - v[l,c]
                    v[l,c-1] = 1 - v[l,c-1]
                    h[l,c-1] = 1 - h[l,c-1]
                    h[l+1,c-1] = 1 - h[l+1,c-1]
                    taken[l,c-1] = 1 - taken[l,c-1]
                    taken[l+1,c-1] = 1 - taken[l+1,c-1]
                    # For real time display in a txt file
                    #UpdateTmp("realTimeGeneration.txt", game, h, v)
                    #sleep(0.3)
                elseif temp == 1 && h[l,c-1] == 1 && rand(1:random2) == 1
                    v[l,c] = 1 - v[l,c]
                    v[l,c-1] = 1 - v[l,c-1]
                    h[l,c-1] = 1 - h[l,c-1]
                    h[l+1,c-1] = 1 - h[l+1,c-1]
                    taken[l,c] = 1 - taken[l,c]
                    taken[l+1,c-1] = 1 - taken[l+1,c-1]
                    # For real time display in a txt file
                    #UpdateTmp("realTimeGeneration.txt", game, h, v)
                    #sleep(0.3)
                elseif temp == 1 && h[l+1,c-1] == 1 && rand(1:random2) == 1
                    v[l,c] = 1 - v[l,c]
                    v[l,c-1] = 1 - v[l,c-1]
                    h[l,c-1] = 1 - h[l,c-1]
                    h[l+1,c-1] = 1 - h[l+1,c-1]
                    taken[l+1,c] = 1 - taken[l+1,c]
                    taken[l,c-1] = 1 - taken[l,c-1]
                    # For real time display in a txt file
                    #UpdateTmp("realTimeGeneration.txt", game, h, v)
                    #sleep(0.3)
                else
                    # check right if no change left
                    temp = taken[l,c+1] + taken[l+1,c+1]
                    if temp == 0 && rand(1:random1) == 1
                        v[l,c] = 1 - v[l,c]
                        v[l,c+1] = 1 - v[l,c+1]
                        h[l,c] = 1 - h[l,c]
                        h[l+1,c] = 1 - h[l+1,c]
                        taken[l,c+1] = 1 - taken[l,c+1]
                        taken[l+1,c+1] = 1 - taken[l+1,c+1]
                        # For real time display in a txt file
                        #UpdateTmp("realTimeGeneration.txt", game, h, v)
                        #sleep(0.3)
                    elseif temp == 1 && h[l,c] == 1 && rand(1:random2) == 1
                        v[l,c] = 1 - v[l,c]
                        v[l,c+1] = 1 - v[l,c+1]
                        h[l,c] = 1 - h[l,c]
                        h[l+1,c] = 1 - h[l+1,c]
                        taken[l,c] = 1 - taken[l,c]
                        taken[l+1,c+1] = 1 - taken[l+1,c+1]
                        # For real time display in a txt file
                        #UpdateTmp("realTimeGeneration.txt", game, h, v)
                        #sleep(0.3)
                    elseif temp == 1 && h[l+1,c] == 1 && rand(1:random2) == 1
                        v[l,c] = 1 - v[l,c]
                        v[l,c+1] = 1 - v[l,c+1]
                        h[l,c] = 1 - h[l,c]
                        h[l+1,c] = 1 - h[l+1,c]
                        taken[l+1,c] = 1 - taken[l+1,c]
                        taken[l,c+1] = 1 - taken[l,c+1]
                        # For real time display in a txt file
                        #UpdateTmp("realTimeGeneration.txt", game, h, v)
                        #sleep(0.3)
                    end
                end
            end
        end
    end

    # Set the circles wherever possible
    game = Vector{Vector{Int}}(undef, n)
    for j in 1:n
        game[j] = zeros(Int, m)
    end

    for l=1:n, c=1:m
        # Check if black circle
        if (l < n-1 && c < m-1 && h[l,c] + h[l,c+1] + v[l,c] + v[l+1,c] == 4) ||
                (l < n-1 && c > 2 && h[l,c-1] + h[l,c-2] + v[l,c] + v[l+1,c] == 4) ||
                (l > 2 && c < m-1 && h[l,c] + h[l,c+1] + v[l-1,c] + v[l-2,c] == 4) ||
                (l > 2 && c > 2 && h[l,c-1] + h[l,c-2] + v[l-1,c] + v[l-2,c] == 4)
            game[l][c] = 2
            # For real time display in a txt file
            #UpdateTmp("realTimeGeneration.txt", game, h, v)
            #sleep(0.3)
        # Check if white circle
        elseif c > 1 && c < m && h[l,c-1] + h[l,c] == 2 && !(c > 2 && c < m-1 && h[l,c-2] + h[l,c-1] + h[l,c] + h[l,c+1]  == 4)
            game[l][c] = 1
            # For real time display in a txt file
            #UpdateTmp("realTimeGeneration.txt", game, h, v)
            #sleep(0.3)
        elseif l > 1 && l < n && v[l-1,c] + v[l,c] == 2 && !(l > 2 && l < n-1 && v[l-2,c] + v[l-1,c] + v[l,c] + v[l+1,c]  == 4)
            game[l][c] = 1
            # For real time display in a txt file
            #UpdateTmp("realTimeGeneration.txt", game, h, v)
            #sleep(0.3)
        end
    end
    
    sum_circles = sum(1 for i=1:n, j=1:m if game[i][j] == 2; init=0)
    if sum_circles * 100 / (n*m) > 10 || recur > max_recur
        if recur == 0
            return game
        end
        return sum_circles, game
    else
        new_sum, new_game = generateInstance(n,m, recur + 1)
        if new_sum > sum_circles
            game = copy(new_game)
            sum_circles = new_sum
        end
        if recur == 0
            return game
        end
        return sum_circles, game
    end
end 

"""
Generate all the instances

Remark: a grid is generated only if the corresponding output file does not already exist
"""
function generateDataSet()

    # TODO
    #println("In file generation.jl, in method generateDataSet(), TODO: generate an instance")
    
    # For each grid size considered
    for size in [(6,6), (8,8), (10,10), (12,8), (12,10), (12,12), (14,10), (16,16), (20,20), (22,22), (24,24), (26,26), (28,28)]
        # Generate 10 instances
        for instance in 1:10
            fileName = "../data/instance_n" * string(size[1]) * "_m" * string(size[2]) * "_" * string(instance) * ".txt"

            if !isfile(fileName)
                println("-- Generating file " * fileName)

                game = generateInstance(size[1], size[2])
                # Open the output file
                writer = open(fileName, "w")
                # For each cell (l, c) of the grid
                for l in 1:size[1]
                    for c in 1:size[2]
                        # Write its value
                        print(writer, game[l][c])
                        if c != size[2]
                            print(writer, ",")
                        else
                            println(writer, "")
                        end
                    end
                end
                close(writer)
            end 
        end
    end
end



