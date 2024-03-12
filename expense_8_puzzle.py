import sys
import logging
import time
from queue import PriorityQueue
from queue import LifoQueue
from dataclasses import dataclass, field 

class Node:
    def __init__(self, state, parent=None, action=None,stepcost=0,hcost=0,fcost=0):
        self.state = state
        self.parent = parent
        self.action = action
        self.depth = 0
        self.stepcost = stepcost
        self.hcost=hcost
        self.fcost = fcost
        if parent:
            self.depth = parent.depth + 1

    def __str__(self):
        if self.parent is not None:
            return str(self.state) 
        else:
            return str(self.state) 

    def __set__(self, parent):
        self.parent = parent
    
    def __lt__(self, other):
        return self.fcost < other.fcost

    def __gt__(self, other):
        return self.fcost > other.fcost

    def __le__(self, other):
        return (self.fcost  < other.fcost) or (self.fcost  == other.fcost)

    def __ge__(self, other):
        return (self.fcost  > other.fcost) or (self.fcost  == other.fcost)

    def __eq__(self, other):
        return (self.fcost  == other.fcost)

    def __ne__(self, other):
        return not (self.fcost  == other.fcost)

# Function to calculate Heuristic Distance using Manhattan Distance method
def heuristic_dist(start_state, goal_state):
    x, y, p, q = None, None, None, None

    val = 1
    total = 0

    for val in range(9):
        for i in range(3):
            for j in range(3):
                if start_state[i][j] == val:
                    x, y = i, j
                    break

    #print(x,y)

        for i in range(3):
            for j in range(3):
                if goal_state[i][j] == val:
                    p, q = i, j
                    break
        total = total + (val * (abs(p-x)+abs(q-y)))

    #print(p,q)
    
    return total 

# Function to check if a move is valid
def check_valid_move(x, y):
    return 0 <= x < 3 and 0 <= y < 3

# Function to determine direction of the tile movement from the current position 
def get_dir (dx,dy):
    if(dy == 0):
        if(dx == -1):
            dir = "down"
        elif(dx == 1):
            dir = "up"
    elif(dy == -1):
        dir = "right"
    elif(dy == 1):
        dir = "left"
    return dir

# Function to determine expanded/child nodes/state of the current state of 8 puzzle problem
def get_children(node):
    state = node.state
    stepcost = 0
    stepcost_node = int(str(node.stepcost))
    x, y = None, None
    for i in range(3):
        for j in range(3):
            if state[i][j] == 0:
                x, y = i, j
                break
    children = []
    moves = [(-1, 0), (1, 0), (0, -1), (0, 1)]
    for dx, dy in moves:
        new_x, new_y = x + dx, y + dy
        if check_valid_move(new_x, new_y):
            new_state = [row[:] for row in state]
            new_state[x][y], new_state[new_x][new_y] = new_state[new_x][new_y], new_state[x][y]
            stepcost = stepcost_node + int(str(new_state[x][y]))
            
            dir = get_dir(dx,dy)

            actionStr = str("move " + str(new_state[x][y]) + " " + dir)
            tempNode = Node(new_state, parent=node, action=actionStr, stepcost=stepcost)
            tempNode.parent = node;
            children.append(tempNode)

    return children

# Function for  Greedy Search algorithm for an 8 puzzle problem 
def greedy(start_state, goal_state):

    print("Executing Greedy  method")
    if dumpflag == "true": logger.debug("Executing Greedy  method")

    fringe_list = []
    tempnode = Node(start_state)
    visited = set()
    myqueue = PriorityQueue()
    start_node = Node(start_state)
    nodes_popped = 0
    expanded_nodes = 0
    start_node.action = str("start")
    start_node.hcost = heuristic_dist(start_node.state, goal_state) 
    start_node.fcost = start_node.hcost  
    tempStr = "State = " + str(start_node.state) + " Action: "+ str(start_node.action) +  " g(n): " + str(start_node.stepcost) + " d: " +  str(start_node.depth) + " f(n): " + str(start_node.fcost) 
    if dumpflag == "true": logger.debug(tempStr)
    myqueue.put(((start_node.fcost, start_node)))
    nodes_generated = 1
    while myqueue:
        fringe_list.clear()
        while not myqueue.empty():
            priority, node = myqueue.get()
            fringe_list.append(((priority, node)))

        for i in range(len(fringe_list)):
            myqueue.put((fringe_list[i][0], fringe_list[i][1]))
            print_node = fringe_list[i][1]

            if dumpflag == "true": logger.debug("\n")
            while print_node is not None:
                tempStr = "State = " + str(print_node.state) + " Action: "+ str(print_node.action) +  " g(n): " + str(print_node.stepcost) + " d: " +  str(print_node.depth) + " f(n): " + str(print_node.fcost) 
                if dumpflag == "true": logger.debug(tempStr)
                print_node = print_node.parent

        mynode = myqueue.get()[1]
        while tuple(map(tuple,mynode.state)) in visited:
            if dumpflag == "true": logger.debug("NODE IN VISITED\n")
            mynode = myqueue.get()[1]
            nodes_popped = nodes_popped+1

        nodes_popped = nodes_popped+1
        
        if mynode.state == goal_state:
            print_node = mynode
            if mynode.parent is not None :
                 if dumpflag == "true": logger.debug("GOAL FOUND AT :")
                 while print_node is not None:
                     tempStr = "State = " + str(print_node.state) + " Action: "+ str(print_node.action) +  " g(n): " + str(print_node.stepcost) + " d: " +  str(print_node.depth) + " f(n): " + str(print_node.fcost) 
                     if dumpflag == "true": logger.debug(tempStr)
                     print_node = print_node.parent
            print("Nodes Popped = ", nodes_popped)
            print("Nodes Expanded = ",expanded_nodes)
            print("Nodes Generated = ", nodes_generated)
            print("Max Fringe size = ", myqueue.qsize())
            print("Solution Found at depth = ", mynode.depth , "with cost of ", mynode.fcost)
            print_node = mynode
            print_q = []
            if mynode.parent is not None :
                print("Steps:")
                while print_node is not None:
                    print_q.insert(0, print_node.action)
                    print_node = print_node.parent
                print(*print_q, sep = '\n')
            return mynode.depth
        
        print_node = mynode
        if mynode.parent is not None :
            if dumpflag == "true": logger.debug("GENERATING SUCCESSORS TO :")
            while print_node is not None:
                tempStr = "State = " + str(print_node.state) + " Action: "+ str(print_node.action) +  " g(n): " + str(print_node.stepcost) + " d: " +  str(print_node.depth) + " f(n): " + str(print_node.fcost) 
                if dumpflag == "true": logger.debug(tempStr)
                print_node = print_node.parent
        visited.add(tuple(map(tuple,mynode.state)))
        if dumpflag == "true": logger.debug("Closed = "+ str(visited))
        children = get_children(mynode)
        expanded_nodes=expanded_nodes+1
        successor_count = 0

        for child in children:
            successor_count = successor_count + 1
            child.hcost = heuristic_dist(child.state, goal_state) 
            child.fcost = child.hcost 
            myqueue.put(((child.fcost, child)))
        if dumpflag == "true": logger.debug("NUM Of SUCCESSORS GENERATED :" + str(successor_count))
        if dumpflag == "true": logger.debug("\n")
        nodes_generated = nodes_generated + successor_count

    print("Nodes Popped = ", nodes_popped)
    print("Nodes Expanded = ",expanded_nodes)
    print("Nodes Generated = ", nodes_generated)
    print("Max Fringe size = ", myqueue.qsize())
    print("Solution Not Found" )

    return None  # If no solution is found


# Function for A* search algorithm for an 8 puzzle problem
def astar(start_state, goal_state):

    print("Executing A* method")
    if dumpflag == "true": logger.debug("Executing A* method")

    fringe_list = []
    tempnode = Node(start_state)
    visited = set()
    myqueue = PriorityQueue()
    start_node = Node(start_state)
    nodes_popped = 0
    expanded_nodes = 0
    start_node.action = str("start")
    start_node.hcost = heuristic_dist(start_node.state, goal_state) 
    start_node.fcost = start_node.hcost + start_node.stepcost
    tempStr = "State = " + str(start_node.state) + " Action: "+ str(start_node.action) +  " g(n): " + str(start_node.stepcost) + " d: " +  str(start_node.depth) + " f(n): " + str(start_node.fcost) 
    if dumpflag == "true": logger.debug(tempStr)
    myqueue.put(((start_node.fcost, start_node)))
    nodes_generated = 1
    while myqueue:
        fringe_list.clear()
        while not myqueue.empty():
            priority, node = myqueue.get()
            fringe_list.append(((priority, node)))

        for i in range(len(fringe_list)):
            myqueue.put((fringe_list[i][0], fringe_list[i][1]))
            print_node = fringe_list[i][1]

            if dumpflag == "true": logger.debug("\n")
            while print_node is not None:
                tempStr = "State = " + str(print_node.state) + " Action: "+ str(print_node.action) +  " g(n): " + str(print_node.stepcost) + " d: " +  str(print_node.depth) + " f(n): " + str(print_node.fcost) 
                if dumpflag == "true": logger.debug(tempStr)
                print_node = print_node.parent

        mynode = myqueue.get()[1]
        while tuple(map(tuple,mynode.state)) in visited:
            if dumpflag == "true": logger.debug("NODE IN VISITED\n")
            mynode = myqueue.get()[1]
            nodes_popped = nodes_popped+1

        nodes_popped = nodes_popped+1
        
        if mynode.state == goal_state:
            print_node = mynode
            if mynode.parent is not None :
                 if dumpflag == "true": logger.debug("GOAL FOUND AT :")
                 while print_node is not None:
                     tempStr = "State = " + str(print_node.state) + " Action: "+ str(print_node.action) +  " g(n): " + str(print_node.stepcost) + " d: " +  str(print_node.depth) + " f(n): " + str(print_node.fcost) 
                     if dumpflag == "true": logger.debug(tempStr)
                     print_node = print_node.parent
            print("Nodes Popped = ", nodes_popped)
            print("Nodes Expanded = ",expanded_nodes)
            print("Nodes Generated = ", nodes_generated)
            print("Max Fringe size = ", myqueue.qsize())
            print("Solution Found at depth = ", mynode.depth , "with cost of ", mynode.fcost)
            print_node = mynode
            print_q = []
            if mynode.parent is not None :
                print("Steps:")
                while print_node is not None:
                    print_q.insert(0, print_node.action)
                    print_node = print_node.parent
                print(*print_q, sep = '\n')
            return mynode.depth
        
        print_node = mynode
        if mynode.parent is not None :
            if dumpflag == "true": logger.debug("GENERATING SUCCESSORS TO :")
            while print_node is not None:
                tempStr = "State = " + str(print_node.state) + " Action: "+ str(print_node.action) +  " g(n): " + str(print_node.stepcost) + " d: " +  str(print_node.depth) + " f(n): " + str(print_node.fcost) 
                if dumpflag == "true": logger.debug(tempStr)
                print_node = print_node.parent
        visited.add(tuple(map(tuple,mynode.state)))
        if dumpflag == "true": logger.debug("Closed = "+ str(visited))
        children = get_children(mynode)
        expanded_nodes=expanded_nodes+1
        successor_count = 0

        for child in children:
            successor_count = successor_count + 1
            child.hcost = heuristic_dist(child.state, goal_state) 
            child.fcost = child.hcost + child.stepcost
            myqueue.put(((child.fcost, child)))
        if dumpflag == "true": logger.debug("NUM Of SUCCESSORS GENERATED :" + str(successor_count))
        if dumpflag == "true": logger.debug("\n")
        nodes_generated = nodes_generated + successor_count

    print("Nodes Popped = ", nodes_popped)
    print("Nodes Expanded = ",expanded_nodes)
    print("Nodes Generated = ", nodes_generated)
    print("Max Fringe size = ", myqueue.qsize())
    print("Solution Not Found" )

    return None  # If no solution is found

# Function for Uniform Cost search of an 8 puzzle Problem
def ucs(start_state, goal_state):

    print("Executing ucs method")

    fringe_list = []
    tempnode = Node(start_state)
    visited = set()
    myqueue = PriorityQueue()
    start_node = Node(start_state)
    nodes_popped = 0;
    successor_count = 0
    expanded_nodes = 0
    start_node.action = str("start")
    myqueue.put(((0,start_node)))
    nodes_generated = 1

    tempStr = "State = " + str(start_node.state) + " Action: "+ str(start_node.action) +  " g(n): " + str(start_node.stepcost) + " d: " +  str(start_node.depth) + " f(n): " + str(start_node.fcost) 
    if dumpflag == "true": logger.debug(tempStr)
    while myqueue:
        mynode = myqueue.get()[1]
        fringe_list.clear()
        while not myqueue.empty():
            priority, node = myqueue.get()
            fringe_list.append(((priority, node)))

        for i in range(len(fringe_list)):
            myqueue.put((fringe_list[i][0], fringe_list[i][1]))
            print_node = fringe_list[i][1]

            if dumpflag == "true": logger.debug("\n")
            while print_node is not None:
                tempStr = "State = " + str(print_node.state) + " Action: "+ str(print_node.action) +  " g(n): " + str(print_node.stepcost) + " d: " +  str(print_node.depth) + " f(n): " + str(print_node.fcost) 
                if dumpflag == "true": logger.debug(tempStr)
                print_node = print_node.parent
        while tuple(map(tuple,mynode.state)) in visited:
            #print("NODE IN VISITED\n")
            mynode = myqueue.get()[1]
            nodes_popped = nodes_popped+1

        nodes_popped = nodes_popped+1
        
        if mynode.state == goal_state:
            if dumpflag == "true": logger.debug("GOAL FOUND AT :")
            print_node = mynode
            while print_node is not None:
                tempStr = "State = " + str(print_node.state) + " Action: "+ str(print_node.action) +  " g(n): " + str(print_node.stepcost) + " d: " +  str(print_node.depth) + " f(n): " + str(print_node.fcost) 
                if dumpflag == "true": logger.debug(tempStr)
                print_node = print_node.parent
            print("Nodes Popped = ", nodes_popped)
            print("Nodes Expanded = ",expanded_nodes)
            print("Nodes Generated = ", nodes_generated)
            print("Max Fringe size = ", myqueue.qsize())
            print("Solution Found at depth = ", mynode.depth , "with cost of ", mynode.stepcost)
            print_node = mynode
            print_q = []
            if mynode.parent is not None :
                print("Steps:")
                while print_node is not None:
                    print_q.insert(0, print_node.action)
                    print_node = print_node.parent
                print(*print_q, sep = '\n')
            return mynode.depth
        print_node = mynode
        if mynode.parent is not None :
            if dumpflag == "true": logger.debug("GENERATING SUCCESSORS TO :")
            while print_node is not None:
                tempStr = "State = " + str(print_node.state) + " Action: "+ str(print_node.action) +  " g(n): " + str(print_node.stepcost) + " d: " +  str(print_node.depth) + " f(n): " + str(print_node.fcost) 
                if dumpflag == "true": logger.debug(tempStr)
                print_node = print_node.parent
        visited.add(tuple(map(tuple,mynode.state)))
        if dumpflag == "true": logger.debug("Closed = "+ str(visited))
        children = get_children(mynode)
        successor_count = 0
        expanded_nodes=expanded_nodes+1
        for child in children:
            myqueue.put(((child.stepcost, child)))
            successor_count = successor_count + 1
            print_node = child
            print_node = print_node.parent
    
        nodes_generated = nodes_generated + successor_count
        if dumpflag == "true": logger.debug("NUM Of SUCCESSORS GENERATED :" + str(successor_count))
        if dumpflag == "true": logger.debug("\n")
    return None  # If no solution is found


# Function for Breadth First Search algorithm of an 8 puzzle problem
def bfs(start_state, goal_state):

    print("Executing bfs method")

    #fringe_list = []
    expanded_nodes=0
    visited = set()
    start_node = Node(start_state)
    nodes_popped = 0
    start_node.action = str("start")
    successor_count = 0
    queue = [start_node]
    nodes_generated=1;
    nodes_popped = 0;
    stepcost = 0;
    tempStr = "State = " + str(start_node.state) + " Action: "+ str(start_node.action) +  " g(n): " + str(start_node.stepcost) + " d: " +  str(start_node.depth) + " f(n): " + str(start_node.fcost) 
    if dumpflag == "true": logger.debug(tempStr)
    while queue:
        node = queue.pop(0)
        nodes_popped = nodes_popped + 1
        for i_node in queue:
            print_node = i_node
            if dumpflag == "true": logger.debug("\n")
            while print_node is not None:
                tempStr = "State = " + str(print_node.state) + " Action: "+ str(print_node.action) +  " g(n): " + str(print_node.stepcost) + " d: " +  str(print_node.depth) + " f(n): " + str(print_node.fcost) 
                if dumpflag == "true": logger.debug(tempStr)
                print_node = print_node.parent

        while(tuple(map(tuple,node.state))) in visited:
                #print("NODE IN VISITED\n");
                node = queue.pop(0)
                nodes_popped = nodes_popped+1
        if node.state == goal_state:
            #print("Nodes generated = ", nodes_generated , "Nodes popped = ", nodes_popped)
            if dumpflag == "true": logger.debug("GOAL FOUND AT :")
            print_node = node
            while print_node is not None:
                tempStr = "State = " + str(print_node.state) + " Action: "+ str(print_node.action) +  " g(n): " + str(print_node.stepcost) + " d: " +  str(print_node.depth) + " f(n): " + str(print_node.fcost) 
                if dumpflag == "true": logger.debug(tempStr)
                print_node = print_node.parent
            print("Nodes Popped = ", nodes_popped)
            print("Nodes Expanded = ",expanded_nodes)
            print("Nodes Generated = ", nodes_generated)
            print("Max Fringe size = ", len(queue))
            print("Solution Found at depth = ", node.depth , "with cost of ", node.fcost)
            print_node = node
            print_q = []
            if node.parent is not None:
                print("Steps:")
                while print_node is not None:
                    print_q.insert(0, print_node.action)
                    print_node = print_node.parent
                print(*print_q, sep = '\n')
            return node.depth
        print_node = node 
        if node.parent is not None :
            if dumpflag == "true": logger.debug("GENERATING SUCCESSORS TO :")
            while print_node is not None:
                tempStr = "State = " + str(print_node.state) + " Action: "+ str(print_node.action) +  " g(n): " + str(print_node.stepcost) + " d: " +  str(print_node.depth) + " f(n): " + str(print_node.fcost) 
                if dumpflag == "true": logger.debug(tempStr)
                print_node = print_node.parent
        visited.add(tuple(map(tuple,node.state)))
        if dumpflag == "true": logger.debug("Closed = "+ str(visited))
        expanded_nodes=expanded_nodes+1
        children = get_children(node)
        successor_count = 0
        for child in children:
            queue.append(child)
            successor_count = successor_count +1
            print_node = child
            #print("\n")
            #while print_node is not None:
            #    print(print_node.state,print_node.action,print_node.stepcost,print_node.depth,print_node.fcost)
            #    print_node = print_node.parent

        nodes_generated = nodes_generated + successor_count 
        if dumpflag == "true": logger.debug("NUM Of SUCCESSORS GENERATED :" + str(successor_count))
        if dumpflag == "true": logger.debug("\n")
    return None  # If no solution is found

# Function for Depth First Search algorithm of an 8 puzzle problem 
def dfs(start_state, goal_state):

    print("Executing dfs method")

    dfs_queue = LifoQueue()
    fringe_list = []
    visited = set()
    start_node = Node(start_state)
    nodes_popped = 0
    expanded_nodes = 0
    start_node.action = str("start")
    #print(start_node.state,  start_node.action, start_node.stepcost,start_node.depth, nodes_popped)

    dfs_queue.put(start_node) 
    nodes_generated=1;
    nodes_popped = 0;
    stepcost = 0;
    tempStr =  "State = " + str(start_node.state) + " Action: "+ str(start_node.action) +  " g(n): " + str(start_node.stepcost) + " d: " +  str(start_node.depth) + " f(n): " + str(start_node.fcost)
    if dumpflag == "true": logger.debug(tempStr)
    while dfs_queue:
        fringe_list.clear()
        #node = dfs_queue.get()
        #nodes_popped = nodes_popped + 1
        while not dfs_queue.empty():
            node = dfs_queue.get()
            fringe_list.append(node)

        for i in range(len(fringe_list),0, -1):
            dfs_queue.put(fringe_list[i-1])
            print_node = fringe_list[i-1]
            if dumpflag == "true": logger.debug("\n")
            while print_node is not None:
                tempStr = "State = " + str(print_node.state) + " Action: "+ str(print_node.action) +  " g(n): " + str(print_node.stepcost) + " d: " +      str(print_node.depth) + " f(n): " + str(print_node.fcost)
                if dumpflag == "true": logger.debug(tempStr)
                print_node = print_node.parent
        node = dfs_queue.get()
        while(tuple(map(tuple,node.state))) in visited:
                node = dfs_queue.get()
                nodes_popped = nodes_popped+1
        nodes_popped = nodes_popped + 1

        if node.state == goal_state:
            #print("Nodes generated = ", nodes_generated , "Nodes popped = ", nodes_popped)
            if dumpflag == "true": logger.debug("GOAL FOUND AT :")
            print_node = node
            while print_node is not None:
                tempStr = "State = " + str(print_node.state) + " Action: "+ str(print_node.action) +  " g(n): " + str(print_node.stepcost) + " d: " +  str(print_node.depth) + " f(n): " + str(print_node.fcost) 
                if dumpflag == "true": logger.debug(tempStr)
                print_node = print_node.parent
            print("Nodes Popped = ", nodes_popped)
            print("Nodes Expanded = ",expanded_nodes)
            print("Nodes Generated = ", nodes_generated)
            print("Max Fringe size = ", dfs_queue.qsize())
            #print("Max Fringe size = ", len(dfs_queue))
            print("Solution Found at depth = ", node.depth , "with cost of ", node.fcost)
            print_node = node
            print_q = []
            if node.parent is not None:
                print("Steps:")
                while print_node is not None:
                    print_q.insert(0, print_node.action)
                    print_node = print_node.parent
                print(*print_q, sep = '\n')
            return node.depth

        print_node = node 
        if node.parent is not None:
            if dumpflag == "true": logger.debug("GENERATING SUCCESSORS TO :")
            while print_node is not None:
                tempStr = "State = " + str(print_node.state) + " Action: "+ str(print_node.action) +  " g(n): " + str(print_node.stepcost) + " d: " +  str(print_node.depth) + " f(n): " + str(print_node.fcost) 
                if dumpflag == "true": logger.debug(tempStr)
                print_node = print_node.parent
        visited.add(tuple(map(tuple,node.state)))
        if dumpflag == "true": logger.debug("Closed = "+ str(visited))
        expanded_nodes=expanded_nodes+1
        children = get_children(node)
        successor_count = 0
        for child in children:
            dfs_queue.put(child)
            successor_count = successor_count +1
            print_node = child

        nodes_generated = nodes_generated + successor_count 
        if dumpflag == "true": logger.debug("NUM Of SUCCESSORS GENERATED :" + str(successor_count))
        if dumpflag == "true": logger.debug("\n")
    return None  # If no solution is found

# Function to read puzzle from the file given through the command prompt
def read_puzzle_file(file_path):
    puzzle = []
    with open(file_path, 'r') as file:
        for line in file:
            if "END OF FILE" in line:
                break
            row = line.split()
            if len(row) != 3:
                raise ValueError("Each row of the puzzle should contain exactly 3 numbers.")
            puzzle.append([int(num) for num in row])
    if len(puzzle) != 3:
        raise ValueError("The puzzle should have exactly 3 rows.")
    return puzzle

if __name__ == '__main__':

    if len(sys.argv) < 3:
        print("Usage: python expense_8_puzzle.py <start_state_file> <goal_state_file> <method> <dumpflag>")
        sys.exit(1)

    method = "a*"
    dumpflag = "false" 

    if len(sys.argv) == 3:
        start_state_file = sys.argv[1]
        goal_state_file = sys.argv[2]
    elif len(sys.argv)== 4:
        start_state_file = sys.argv[1]
        goal_state_file = sys.argv[2]
        if((sys.argv[3] == "true") or (sys.argv[3] == "false")):
            dumpflag = sys.argv[3]
        else: 
            method = sys.argv[3]
            dumpflag = "false" 
    elif len(sys.argv)== 5:
        start_state_file = sys.argv[1]
        goal_state_file = sys.argv[2]
        method  = sys.argv[3]
        dumpflag = sys.argv[4]

    print("Method: " + method + " dumpflag: " + dumpflag);

    try:
        start_state = read_puzzle_file(start_state_file)
        goal_state = read_puzzle_file(goal_state_file)
        
        if dumpflag == "true": 
           date = time.strftime("%d-%m-%Y")
           timeCreated = time.strftime("%H-%M-%S")
           logFileName = "trace-"+date+ "-"+timeCreated+ ".txt"
           logging.basicConfig(filename=logFileName, format='', filemode='w')
           logger = logging.getLogger()
           logger.setLevel(logging.DEBUG)

        if method == "bfs":
            depth = bfs(start_state, goal_state)
        elif method == "dfs":
            depth = dfs(start_state, goal_state)
        elif method == "ucs":
            depth = ucs(start_state, goal_state)
        elif method == "greedy":
            depth = greedy(start_state, goal_state)
        elif method == "a*":
            depth = astar(start_state, goal_state)
        else:
            depth = astar(start_state, goal_state)


        if depth is not None:
            print("Depth of solution:", depth)
        else:
            print("No solution found.")
    except FileNotFoundError:
        print("File not found.")
    except ValueError as e:
        print(f"Invalid input: {str(e)}")

