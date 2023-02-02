#!/usr/bin/python3

# A utility function to print solution matrix sol
def printSolution( sol ):
    print("\n\nPossible Solution :\n")
    for i in sol:
        for j in i:
            print(str(j) + " ", end ="")
        print("")
 
# A utility function to check if x, y is valid
# index for N * N Maze
def isSafe( maze, x, y, goal):
     
    if x >= 0 and x < goal[0] and y >= 0 and y < goal[1] and maze[x][y] == 1:
        return True
     
    return False
 
""" This function solves the Maze problem using Backtracking.
    It mainly uses solveMazeUtil() to solve the problem. It
    returns false if no path is possible, otherwise return
    true and prints the path in the form of 1s. Please note
    that there may be more than one solutions, this function
    prints one of the feasable solutions. """
def solveMaze( maze , goal):

    # Creating a 4 * 4 2-D list
    sol = [ [ 0 for j in range(goal[0]) ] for i in range(goal[1]) ]
     
    if solveMazeUtil(maze, 1, 1, sol, goal) == False:
        #print("Solution doesn't exist")
        return False
     
    printSolution(sol)
    return True
     
# A recursive utility function to solve Maze problem
def solveMazeUtil(maze, x, y, sol, goal):
     
    # if (x, y is goal) return True
    if x == goal[0]-2 and y == goal[1]-2 and maze[x][y]== 1:
        sol[x][y] = 1
        return True
         
    # Check if maze[x][y] is valid
    if isSafe(maze, x, y, goal) == True:
        # Check if the current block is already part of solution path.   
        if sol[x][y] == 1:
            return False
           
        # mark x, y as part of solution path
        sol[x][y] = 1
         
        # Move forward in x direction
        if solveMazeUtil(maze, x + 1, y, sol, goal) == True:
            return True
             
        # If moving in x direction doesn't give solution
        # then Move down in y direction
        if solveMazeUtil(maze, x, y + 1, sol, goal) == True:
            return True
           
        # If moving in y direction doesn't give solution then
        # Move back in x direction
        if solveMazeUtil(maze, x - 1, y, sol, goal) == True:
            return True
             
        # If moving in backwards in x direction doesn't give solution
        # then Move upwards in y direction
        if solveMazeUtil(maze, x, y - 1, sol, goal) == True:
            return True
         
        # If none of the above movements work then
        # BACKTRACK: unmark x, y as part of solution path
        sol[x][y] = 0
        return False

