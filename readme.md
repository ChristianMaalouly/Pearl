Commands for Pearl

In a julia terminal
cd("directory of src folder")
include("resolution.jl")

For linear program:
solve(readInputFile("../data/instanceTest.txt"))
Uncomment the displaySolution function at the very end of solve function to have the solution displayed

For heuristic:
heuristicSolve(readInputFile("../data/instanceTest.txt"))
Uncomment the displayTmp function at the very end of heuristicSolve function to have the solution displayed

To generate a data set
generateDataSet()

To solve all the grids in the folder /data
solveDataSet()

To create a latex file for the results
resultsArray("filename.tex")
