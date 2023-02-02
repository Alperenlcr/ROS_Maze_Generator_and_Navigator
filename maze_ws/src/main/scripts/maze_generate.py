#!/usr/bin/python3

import random



class Cell:
    """Cell class that defines each walkable Cell on the grid"""
    def __init__(self, x: int, y: int):
        self.x = x
        self.y = y
        self.visited = False
        self.walls = [True, True, True, True] # Left, Right, Up, Down


    def getChildren(self, grid: list) -> list:
        """Check if the Cell has any surrounding unvisited Cells that are walkable"""
        a = [(1, 0), (-1,0), (0, 1), (0, -1)]
        children = []
        for x, y in a:
            if self.x+x in [len(grid), -1] or self.y+y in [-1, len(grid)]:
                continue
            
            child = grid[self.y+y][self.x+x]
            if child.visited:
                continue
            children.append(child)
        return children


def removeWalls(current: Cell, choice: Cell):
    """Removeing the wall between two Cells"""
    if choice.x > current.x:     
        current.walls[1] = False
        choice.walls[0] = False
    elif choice.x < current.x:
        current.walls[0] = False
        choice.walls[1] = False
    elif choice.y > current.y:
        current.walls[3] = False
        choice.walls[2] = False
    elif choice.y < current.y:
        current.walls[2] = False
        choice.walls[3] = False


def drawWalls(grid: list, binGrid: list) -> list:
    """Draw existing walls around Cells"""
    for yindex, y in enumerate(grid):
        for xindex, x in enumerate(y):
            for i, w in enumerate(x.walls):
                if i == 0 and w:
                    binGrid[yindex*2+1][xindex*2] = 0
                if i == 1 and w:
                    binGrid[yindex*2+1][xindex*2+2] = 0
                if i == 2 and w:
                    binGrid[yindex*2][xindex*2+1] = 0
                if i == 3 and w:
                    binGrid[yindex*2+2][xindex*2+1] = 0
    return binGrid


def drawBorder(grid: list) -> list:
    """Draw a border around the maze"""
    length = len(grid)
    for row in grid:
        row[0] = row[length-1] = 0
        
    grid[0] = grid[length-1] = [0] * length
    return grid


def finalizeMaze(grid: list, size, koordinatlar):
    """Draw the maze using ASCII characters and display the maze"""
    binGrid = []
    length = len(grid)*2+1
    for x in range(length):
        if x % 2 == 0:
            binGrid.append([1 if x % 2 != 0 else 0 for x in range(length)])
        else:
            binGrid.append([1] * length)
    
    binGrid = drawWalls(grid, binGrid)
    binGrid = drawBorder(binGrid)
    for x in range(len(binGrid)):
        for y in range(len(binGrid[0])):
            if binGrid[x][y] == 0:
                koordinatlar.append([x,y])
    return binGrid



def mainn(size, koordinatlar):
    grid = [[Cell(x, y) for x in range(size)] for y in range(size)]
    current = grid[0][0]
    stack = []
    # Main loop to generate the maze
    while True:
        current.visited = True
        children = current.getChildren(grid)

        if children:
            choice = random.choice(children)
            choice.visited = True

            stack.append(current)

            removeWalls(current, choice)

            current = choice

        elif stack:
            current = stack.pop()
        else:
            break

    return finalizeMaze(grid,size, koordinatlar)
