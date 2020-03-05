
''''
author: Shreya Gummadi
'''

import numpy as np
import ast
from copy import deepcopy
init_node = input("Enter initial node:")
init_node = ast.literal_eval(init_node) #ast module helps to take a 2D array input from user
init_node = np.array(init_node)
#init_node = np.array(([1,2,3],[0,4,6],[7,5,8]))
goal_node = np.array([[1,2,3],[4,5,6],[7,8,0]])


'''finds the posistion of the blank tile'''
def BlankTileLocation(node):
    for i in range(3):
        for j in range(3):
            if (node[i][j] == np.min(node)):
                return i,j

'''moves the blank tile left'''
def ActionMoveLeft(curr_node):
    node = deepcopy(curr_node) # if deepcopy is not used python makes changes to the node and stores it, it doesn't store a separate copy
    i,j = BlankTileLocation(node)
    if j ==0:
        return False,node
    else:
        node[i,j-1],node[i,j] = node[i,j],node[i,j-1]
        return True,node

'''moves the blank tile right'''
def ActionMoveRight(curr_node):
    node = deepcopy(curr_node)
    i,j = BlankTileLocation(node)
    if j ==2:
        return False,node
    else:
        node[i,j+1],node[i,j] = node[i,j],node[i,j+1]
        return True,node


'''moves the blank tile down'''
def ActionMoveDown(curr_node):
    node = deepcopy(curr_node)
    i,j = BlankTileLocation(node)
    if i==2:
        return False,node
    else:
        node[i+1,j],node[i,j]= node[i,j],node[i+1,j]
        return True,node

'''moves the blank tile up'''
def ActionMoveUp(curr_node):
    node = deepcopy(curr_node)
    i,j = BlankTileLocation(node)
    if i==0:
        return False,node
    else:
        node[i-1,j],node[i,j] = node[i,j],node[i-1,j]
        return True,node

''' checks if the node is visited or not'''
def belongs(node,visited):
	x = False
	for i in range(len(visited)):
		if (node==visited[i]).all() == True:
		       x = True
		       return x
		       break
	return x       

''' takes the initial and final nodes and performs bfs'''
def Solver(start, goal):

    visited = []
    visitedinfo =[]
    queue = [[start]]
    if (start == goal).all():
        return "Solved!"
    i =0
    done =0
    while queue:
        path = queue.pop(0)
        curr_node = path[-1]
        if belongs(curr_node,visited) == False :
             neighbours = []
             statusdown, nodedown = ActionMoveDown(curr_node)
             if statusdown == True:
             	neighbours.append(nodedown)
             statusup, nodeup = ActionMoveUp(curr_node)
             if statusup == True:
             	neighbours.append(nodeup)
             statusleft, nodeleft = ActionMoveLeft(curr_node)
             if statusleft == True:
             	neighbours.append(nodeleft)
             statusright, noderight = ActionMoveRight(curr_node)
             if statusright == True:
             	neighbours.append(noderight)
            
             for neighbour in neighbours:
                new_path = list(path)
                new_path.append(neighbour)
                queue.append(new_path)
                if (neighbour == goal).all():
                    done =1
                    
             visited.append(curr_node)
             i = i+1
             try :
             	visitedinfo.append({"Parent " :path[-2]  , "Cost " : 0,"Number" : i , }) 
             except :
                visitedinfo.append({"Parent " : "NULL" , "Cost " : 0,"Number" : i , }) 
             if done ==1 :   
             	return new_path , visited, visitedinfo
  
    return "Can't be solved!"


path ,visited , visitedinfo = Solver(init_node , goal_node)
if isinstance(path, str) :
	print(path)
else :
    nodepath = open('nodepath.txt','w')
    nodepath.write("OPTIMAL PATH\n")
    for i in range(len(path)):
        nodepath.write("\nStep " + str(i) + " :\n")
        nodepath.write(str(path[i])+'\n')
   
if len(visited ) == 0:
    print("None Visited")
    
else :
    nodeset = open('nodeset.txt','w')
    nodeinfo = open('nodeinfo.txt','w')
    nodeset.write("Visited Nodes\n")
    nodeinfo.write("Nodes Info\n")
    for i in range(len(visited)):
        nodeset.write('\n'+str(visited[i])+'\n')
        nodeinfo.write('\n'+str(visitedinfo[i])+'\n')

