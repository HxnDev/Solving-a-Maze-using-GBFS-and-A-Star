#!/usr/bin/env python
# coding: utf-8

# In[37]:


#!/usr/bin/env python
# coding: utf-8

# In[5]:


##############################################################################################################################
###         Hassan Shahzad
###         CS-D
###         Artificial Intelligence (Assignment # 1)
###         FAST-NUCES
###         chhxnshah@gmail.com
##############################################################################################################################


import sys
import copy                                     # Will be used to deep copy

################################################## GLOBAL VARIABLES ##########################################################

agent = "A"                                     # Will be used later to find index of agent
goal_index = [(12,19)]                          # Storing the goal index of agent

##CREATING THE MAZE##
## For this we will use 2D arrays and graphs
## In the maze, " " will represent empty cells while "#" will represent blocked cells

rows , cols = (20,20)                           # 12 rows and 12 columns
maze = []                                       # Declaring variables

for i in range(rows):                           # Loop iterating through rows
    col = []
    for j in range(cols):                       # Loop iterating through columns
        col.append("*")                         # Initially storing "#" to show every cell is blocked.
    maze.append(col)


##############################################################################################################################

################################################# FILLING THE MAZE ###########################################################
    
    
    
## Now the empty spaces are filled according to the given space
## I was exhausted so i decided to simply hardcode it instead of mapping

maze[14][1] = "-"
maze[14][2] = "-"
maze[14][3] = "-"
maze[3][2] = "-"
maze[4][2] = "-"
maze[5][2] = "-"
maze[6][2] = "-"
maze[7][2] = "-"
maze[11][2] = "-"
maze[12][2] = "-"
maze[13][2] = "-"
maze[15][2] = "-"
maze[16][2] = "-"
maze[3][3] = "-"
maze[3][4] = "-"
maze[3][5] = "-"
maze[3][6] = "-"
maze[3][7] = "-"
maze[3][8] = "-"
maze[3][9] = "-"
maze[3][10] = "-"
maze[3][13] = "-"
maze[3][14] = "-"
maze[3][15] = "-"
maze[3][16] = "-"
maze[3][17] = "-"
maze[3][18] = "-"
maze[3][19] = "-"
maze[7][3] = "-"
maze[8][3] = "-"
maze[9][3] = "-"
maze[10][3] = "-"
maze[11][3] = "-"
maze[16][3] = "-"
maze[4][10] = "-"
maze[4][13] = "-"
maze[4][17] = "-"
maze[5][10] = "-"
maze[5][11] = "-"
maze[5][12] = "-"
maze[5][13] = "-"
maze[5][17] = "-"
maze[5][18] = "-"
maze[5][19] = "-"
maze[6][10] = "-"
maze[7][10] = "-"
maze[8][10] = "-"
maze[9][10] = "-"
maze[10][10] = "-"
maze[11][10] = "-"
maze[9][9] = "-"
maze[9][8] = "-"
maze[10][8] = "-"
maze[11][8] = "-"
maze[12][8] = "-"
maze[13][8] = "-"
maze[14][8] = "-"
maze[15][8] = "-"
maze[16][8] = "-"
maze[17][8] = "-"
maze[10][6] = "-"
maze[13][6] = "-"
maze[14][6] = "-"
maze[15][6] = "-"
maze[16][6] = "-"
maze[7][5] = "-"
maze[10][5] = "-"
maze[11][5] = "-"
maze[12][5] = "-"
maze[13][5] = "-"
maze[16][5] = "-"
maze[7][4] = "-"
maze[12][4] = "-"
maze[13][4] = "-"
maze[16][4] = "-"
maze[11][11] = "-"
maze[11][12] = "-"
maze[11][13] = "-"
maze[11][14] = "-"
maze[7][13] = "-"
maze[7][14] = "-"
maze[7][15] = "-"
maze[7][16] = "-"
maze[7][17] = "-"
maze[7][19] = "-"
maze[6][13] = "-"
maze[6][17] = "-"
maze[6][19] = "-"
maze[8][17] = "-"
maze[8][19] = "-"
maze[9][19] = "-"
maze[10][16] = "-"
maze[10][17] = "-"
maze[10][18] = "-"
maze[10][19] = "-"
maze[12][16] = "-"
maze[12][17] = "-"
maze[12][18] = "-"
maze[12][19] = "-"
maze[11][16] = "-"
maze[13][16] = "-"
maze[14][18] = "-"
maze[14][19] = "-"
maze[12][14] = "-"
maze[13][14] = "-"
maze[14][14] = "-"
maze[15][14] = "-"
maze[16][14] = "-"
maze[15][15] = "-"
maze[15][16] = "-"
maze[15][17] = "-"
maze[15][18] = "-"
maze[16][17] = "-"
maze[17][17] = "-"


## Finally the start state
maze[14][0] = "A"

##############################################################################################################################

agent_index = [(index, rows.index(agent)) for index, rows in enumerate(maze) if agent in rows]  # Storing the current index of agent
x = agent_index[0][0]                                                                           # Will store the x-coordinate of agent
y = agent_index[0][1]                                                                           # Will store the y-coordinate of agent

def printMatrix(mat) :                                                                          # Prints the formatted matrix
    for i in range(rows):
        for j in range(cols):
            print(mat[i][j],end = ' ')
        print()

def matrixToIndex(state):                                                                       # Takes matrix and returns index of actor
    temp = state.copy()
    idx = [(index, rows.index(agent)) for index, rows in enumerate(temp) if agent in rows]      # Getting the index of the Agent (A)
    x = idx[0][0]                                                                               # Will store the x-coordinate of agent
    y = idx[0][1]                                                                               # Will store the y-coordinate of agent
    return idx,x,y


##############################################################################################################################

################################################# CREATING NODE CLASS ########################################################

class Node:
    def __init__(self, state, parent, operator, moves):                 # Default Constructor
        self.state = state
        self.parent = parent
        self.operator = operator
        self.moves = moves
        self.m_dist = 0                                                 # Initially Manhatten distance is 0
        
        manhattan_distance(self)

    def __eq__(self, other):                                            # Comparing two nodes
        return (type(self) == type(other)) and (self.state == other.state)

    def __lt__(self, other):                                            # Sorting nodes
        return self.m_dist < other.m_dist

def create_node(state, parent, operator, cost):                         # This function creates a node of current state
    return Node(state, parent, operator, cost)


def manhattan_distance(node):
    idx,ax,ay = matrixToIndex(node.state)
    endx,endy = goal_index[0][0],goal_index[0][1]
    
    distance = abs(ax - endx) + abs(ay -endy)
    node.m_dist = distance


def expand_node(node,n, curr_algo):                                             # This function performs all possible operations
    expanded_nodes = []
   
    temp_state1 = move_up(node.state,n)
    
    if (temp_state1 is not None):
        if (curr_algo == "gbfs"):
            temp_node1 = create_node(temp_state1,node,"up",node.moves+1)        # The state is expanded with upward operation
        elif (curr_algo == "a_star"):
            temp_node1 = create_A_node(temp_state1,node,"up",node.g+1,0,0)      # The state is expanded with upward operation
        
        expanded_nodes.append(temp_node1)                                       # Appending the expanded nodes in the list

    temp_state2 = move_left(node.state,n)
    
    if (temp_state2 is not None):
        if (curr_algo == "gbfs"):
            temp_node2 = create_node(temp_state2,node,"left",node.moves+1)    # The state is expanded with upward operation
        elif (curr_algo == "a_star"):
            temp_node2 = create_A_node(temp_state2,node,"left",node.g+1,0,0)  # The state is expanded with upward operation
        expanded_nodes.append(temp_node2)                                     # Appending the expanded nodes in the list
    
    temp_state3 = move_right(node.state,n)
    
    if (temp_state3 is not None):
        if (curr_algo == "gbfs"):
            temp_node3 = create_node(temp_state3,node,"right",node.moves+1)   # The state is expanded with upward operation
        elif (curr_algo == "a_star"):
            temp_node3 = create_A_node(temp_state3,node,"left",node.g+1,0,0)  # The state is expanded with upward operation
        expanded_nodes.append(temp_node3)                                     # Appending the expanded nodes in the list
    
    
    temp_state = move_down(node.state,n)                              
    
    if (temp_state is not None):
        if (curr_algo == "gbfs"):
            temp_node = create_node(temp_state,node,"down",node.moves+1)     # The state is expanded with downward operation
        elif (curr_algo == "a_star"):
            temp_node = create_A_node(temp_state,node,"left",node.g+1,0,0)   # The state is expanded with upward operation
        expanded_nodes.append(temp_node)                                     # Appending the expanded nodes in the list       

    return expanded_nodes



def move_left(state,n):
    swap = copy.deepcopy(state)
    idx,x,y = matrixToIndex(swap)                                        # Returning index of actor

    if (swap[x][y-1] == "*" or y <= 0):                                  # Checks for unallowed moves 
        return None
    else:
        swap[x][y-1] , swap[x][y] = swap[x][y] , swap[x][y-1]            # Moving the agent one cell left
        return swap

def move_right(state,n):
    swap = copy.deepcopy(state)
    idx,x,y = matrixToIndex(swap)                                        # Returning index of actor
    
    if (y >= n-1 or swap[x][y+1] == "*"):                                # Checks for unallowed moves
        return None
    else:
        swap[x][y+1] , swap[x][y] = swap[x][y] , swap[x][y+1]            # Moving the agent one cell left
        return swap

def move_up(state,n):

    swap = copy.deepcopy(state)
    idx,x,y = matrixToIndex(swap)                                        # Returning index of actor
    
    if (swap[x-1][y] == "*" or x <= 0 ):                                 # Checks for unallowed moves
        return None
    else:
        
        swap[x-1][y] , swap[x][y] = swap[x][y] , swap[x-1][y]            # Moving the agent one cell above
        return swap


def move_down(state,n):

    swap = copy.deepcopy(state)
    idx,x,y = matrixToIndex(swap)                                        # Returning index of actor
    
    if (swap[x+1][y] == "*" or x >= n-1):                                # Checks for unallowed moves
        return None
    else:
        swap[x+1][y] , swap[x][y] = swap[x][y] , swap[x+1][y]            # Moving the agent one cell left
        return swap

##############################################################################################################################


################################################## GBFS ALGORITHM #############################################################

def gbfs(start,n):
    
    tmp = "gbfs"
    temp_count =0
    temp_idx,x1,y1 = matrixToIndex(start)                                # Getting agent's current position

    if (temp_idx == goal_index):
        return [None]
    
    else:
        to_be_expanded = []                                              # Array of all nodes in one level/depth
        visited_nodes = []                                               # Tuple that will contain the nodes already visited
        start_node = create_node(start,None,None,0)                      # Starting node is stored 
        to_be_expanded.append(start_node)                                # Adding first node to the expanding array
        
        while to_be_expanded:
            to_be_expanded.sort()                                        # Sorting the nodes wrt to cost (ascending)            
            current_node = to_be_expanded.pop(0)                         # Getting the node with the smallest cost
            visited_nodes.append(current_node)                           # Adding the index to visited array
            
            new_idx,x2,y2 = matrixToIndex(current_node.state)            # Getting agent's new position
            temp_count+=1
            
            if (new_idx == goal_index):                                  # If goal state is found
                print()
                print("Algorithm Used: GBFS (Greedy Best First Search)")
                print()
                print("Maze Solved!!!")
                print("Final State is as follows: ")
                print()
                printMatrix(current_node.state)
                print()
                print("Number of explorations(total moves) in GBFS = ", temp_count)
                return current_node
        
            else:
                node_array = expand_node(current_node,n,tmp)             # Expanding the neigbours
                
                for node in node_array:                                  # Checking conditions of A*
                    if (node not in to_be_expanded):
                        if (node not in visited_nodes):
                            to_be_expanded.append(node)
                
    return None
    

##############################################################################################################################


##################################################### A* NODE CLASS ##########################################################

class A_Node:                                                  # Node Class for A_Star
        
    def __init__(self,state, parent, operator, g, h, f):       # Initializing the class
        self.state = state
        self.parent = parent
        self.operator = operator
        self.g = g                                             # Distance to start node
        self.m_dist = h                                        # Distance to goal node (heuristic = m_dist)
        self.f = f                                             # Total cost
            
    def __eq__(self, other):                                   # Comparing two nodes
        return (type(self) == type(other)) and (self.state == other.state)

    def __lt__(self, other):                                   # Sorting nodes
        return self.m_dist < other.m_dist

def create_A_node(state, parent, operator, g, h, f):           # This function creates a node of current state
    return A_Node(state, parent, operator, g, h, f)

def In_Open(open, neighbor):                                   # Check if a neighbor should be added to open list
    
    for node in open:
        if (neighbor == node and neighbor.f >= node.f):
            return False
    return True

##############################################################################################################################

################################################## A* ALGORITHM #############################################################

def a_star(start,n):
    
    tmp = "a_star"
    temp_count =0
    cost = 0
    temp_idx,x1,y1 = matrixToIndex(start)                                # Getting agent's current position
    
    if (temp_idx == goal_index):
        return [None]
    else:
    
        to_be_expanded = []                                              # Array of all nodes in one level/depth
        visited_nodes = []                                               # Tuple that will contain the nodes already visited
        
        start_node = create_A_node(start,None,None,0,0,0)                # Starting node is stored
        manhattan_distance(start_node)
        start_node.f = start_node.m_dist + start_node.g
        to_be_expanded.append(start_node)                                # Adding first node to the expanding array

        while to_be_expanded:
            temp_count+=1
            to_be_expanded.sort()
            current_node = to_be_expanded.pop(0)
            visited_nodes.append(current_node)
            cost = current_node.g
            
            new_idx,x2,y2 = matrixToIndex(current_node.state)            # Getting agent's new position
            if (new_idx == goal_index):
                print()
                print("Algorithm Used: A* (A Star Algortihm)")
                print()
                print("Maze Solved!!!")
                print("Final State is as follows: ")
                print()
                printMatrix(current_node.state)
                print()
                print("Number of explorations(total moves) in A* = ", temp_count)
                return current_node,cost
            else:
                node_array = expand_node(current_node,n,tmp)
                
                for node in node_array:
                    manhattan_distance(start_node)
                    start_node.f = start_node.m_dist + start_node.g
                    if(In_Open(to_be_expanded, node) == True):
                        if (node not in visited_nodes):
                            to_be_expanded.append(node)
    
    return None 

##############################################################################################################################

################################################# Implementation of Main done ################################################
def main():
    
    n=20;
    starting_state = maze
    printMatrix(maze)
    print()
    print("#################################################################################################################")
    
    result = gbfs(starting_state,n)
    if result == None:
        print("No solution found")
    elif result == [None]:
        print  ("Start node was the goal!")
    else:
        print ("Total number of moves(cost) needed = ", result.moves)

    print()
    print()
    print("#################################################################################################################")
    print()

    result1,cost1 = a_star(starting_state,n)
    if result1 == None:
        print("No solution found")
    elif result1 == [None]:
        print  ("Start node was the goal!")
    else:
        print ("Total number of moves(cost) needed = ", cost1)
        print()
        


if __name__ == "__main__":
    main()

############################################################################################################################
######################################################### THE END ##########################################################
############################################################################################################################






# In[ ]:




