#Project of Panagiotis Sakellariou (MSCAIDL-0050) for AIDL_A01

#A Program to implement A* Search Algorithm
#based on https://github.com/MAN1986/pyamaze/blob/main/Demos/A-Star/aStarDemo.py

from pyamaze import maze, agent ,COLOR, textLabel
from queue import PriorityQueue
from collections import deque
import numpy as np
import os
from os import listdir
import time

def h_cost(current_cell,target_cell,mode):
    '''Choose between : manhattan, euclidean'''

    x1,y1=current_cell
    x2,y2=target_cell

    if mode == 'manhattan':
        return abs(x1-x2) + abs(y1-y2)      
    # elif mode == 'diagonal':
    #     dx = abs(x1-x2)
    #     dy = abs(y1-y2)
    #     D=1                     #I need the right value for the cell's width    
    #     D2=np.sqrt(2)               
    #     return D*(dx + dy)+(D2- 2*D)*min(dx, dy)
    elif mode == 'euclidean':
        return np.sqrt((x1-x2)**2+(y1-y2)**2)
    else:
        print(f'\nThe {mode} heuristic function is not supported!')
        print()

def A_star(m, start=None, goal=None, heuristic=None):
    '''A* Search Algorithm'''
    if start is None:
        start_cell=(m.rows,m.cols)
    else:
        start_cell=start
    if goal is None:
        goal_cell=(1,1)
    else:
        goal_cell=goal
    
    if heuristic is None:
        mode='manhattan'
    else:
        mode=heuristic


    g_cost={cell:float('inf') for cell in m.grid}
    g_cost[start_cell]=0
    f_cost={cell:float('inf') for cell in m.grid}
    f_cost[start_cell]=h_cost(start_cell,goal_cell,mode) + g_cost[start_cell]

    open_=PriorityQueue()
    open_.put((f_cost[start_cell], h_cost(start_cell,goal_cell,mode), start_cell))
    aPath={}
    searchPath=[start_cell]

    while not open_.empty():
        currCell=open_.get()[2]     #get the cell with the minimum f_cost
        searchPath.append(currCell)
        if currCell==goal_cell:
            break
        for d in 'ESNW':
            if m.maze_map[currCell][d]==True:
                if d=='E':
                    childCell=(currCell[0],currCell[1]+1)
                if d=='W':
                    childCell=(currCell[0],currCell[1]-1)
                if d=='N':
                    childCell=(currCell[0]-1,currCell[1])
                if d=='S':
                    childCell=(currCell[0]+1,currCell[1])

                temp_g_cost=g_cost[currCell]+1
                temp_f_cost=temp_g_cost + h_cost(childCell,goal_cell,mode)

                if temp_f_cost < f_cost[childCell]:
                    g_cost[childCell]=temp_g_cost
                    f_cost[childCell]=temp_f_cost
                    open_.put((f_cost[childCell],h_cost(childCell,goal_cell,mode),childCell))
                    aPath[childCell]=currCell

    fwdPath={}
    cell=goal_cell
    while cell != start_cell:
        fwdPath[aPath[cell]]=cell
        cell=aPath[cell]
    return searchPath, aPath, fwdPath

def find_csv_filenames( path_to_dir, suffix=".csv" ):
    filenames = listdir(path_to_dir)
    return [ filename for filename in filenames if filename.endswith( suffix ) ]


def BFS(m,start=None, goal=None):
    if start is None:
        start_cell=(m.rows,m.cols)
    else:
        start_cell = start
    if goal is None:
        goal_cell=(1,1)
    else:
        goal_cell = goal

    frontier = deque()
    frontier.append(start_cell)
    bfsPath = {}
    explored = [start_cell]
    bSearch=[]

    while len(frontier)>0:
        currCell=frontier.popleft()
        if currCell==m._goal:
            break
        for d in 'ESNW':
            if m.maze_map[currCell][d]==True:
                if d=='E':
                    childCell=(currCell[0],currCell[1]+1)
                elif d=='W':
                    childCell=(currCell[0],currCell[1]-1)
                elif d=='S':
                    childCell=(currCell[0]+1,currCell[1])
                elif d=='N':
                    childCell=(currCell[0]-1,currCell[1])
                if childCell in explored:
                    continue
                frontier.append(childCell)
                explored.append(childCell)
                bfsPath[childCell] = currCell
                bSearch.append(childCell)
    # print(f'{bfsPath}')
    fwdPath={}
    cell=goal_cell
    while cell!=start_cell:
        fwdPath[bfsPath[cell]]=cell
        cell=bfsPath[cell]
    return bSearch,bfsPath,fwdPath


def main():

    m=maze(20,20)
    start=(1,1)
    goal=(18,19) #(m.rows,m.cols)
    heuristic_mode='manhattan'

    m.CreateMaze(goal[0],goal[1], loopPercent=50, saveMaze=True)  #change loopPercent to make maze with less or more loops
    directory = os.getcwd()
    filenames = find_csv_filenames(directory)
    for filename in filenames:
        if 'maze--' in filename:
            my_maze = filename

    a=agent(m, start[0],start[1], shape='arrow', footprints=True)
    road=agent(m, start[0], start[1], shape='square',footprints=True, color=COLOR.cyan)
    backroad=agent(m, goal[0], goal[1], shape='square', footprints=True, color=COLOR.green, goal=start)

    #A*
    searchPath,aPath,fwdPath=A_star(m, start, goal, heuristic_mode)
    m.tracePath({road:searchPath},delay=50)
    m.tracePath({backroad:aPath}, delay=50)
    m.tracePath({a:fwdPath},delay=50)

    l=textLabel(m,'A* Search Algorithm',101)

    print(f'The heuristic function is {heuristic_mode}')
    print(f'A Star Path Length {len(fwdPath)+1}')
    print(f'A Star Search Length {len(searchPath)}')
    print()

    m.run()
    
    ## time.sleep()  

    ## m=maze(20,20)
    ## start=(1,1)
    ## goal=(18,19) #(m.rows,m.cols)

    ## m.CreateMaze(goal[0],goal[1], loadMaze=my_maze)

    # a=agent(m, start[0],start[1], shape='arrow', footprints=True)
    # road=agent(m, start[0], start[1], shape='square',footprints=True, color=COLOR.cyan)
    # backroad=agent(m, goal[0], goal[1], shape='square', footprints=True, color=COLOR.green, goal=start)

    # searchPath,bfsPath,fwdPath=BFS(m, start, goal)

    # m.tracePath({road:searchPath},delay=50)
    # m.tracePath({backroad:bfsPath}, delay=50)
    # m.tracePath({a:fwdPath},delay=50)

    # l=textLabel(m,'Breadth First Search Algorithm',102)
    # print(f'BFS Path Length {len(fwdPath)+1}')
    # print(f'BFS Search Length {len(searchPath)}')
    # m.run()
    
    os.remove(my_maze)
if __name__=='__main__':
    main()   