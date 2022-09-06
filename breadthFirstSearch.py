                                                                                                                         
class Problem:
    """
     This class represents a generic problem to be solved by a search algorithm:
    """
    def __init__(self, init_state, goal_state):
        """declared method to initialise the attributes of the class:"""
        self.init_state = init_state
        self.goal_state = goal_state
    def __str__(self):
        """ declared to override the string object representation in memory and 
        return the string value instead
        """
        return (type(self).__name__ + ": Initial state = " + str(self.init_state) + ", Goal state = " + str(self.goal_state))

    def goal_test(self, state):
        return False
    
    def actions(self, state):
        return None

class TwoDNavProb(Problem):
    """
    This class represents the problem of having a robot in 
    a 2D world with obstacles. The TwoDNavProb inherits some attributes 
    such from the class Problem
    """
    def __init__(self, init_state, goal_state, grid):
        super().__init__(init_state, goal_state)
        self.grid = grid

    def goal_test(self, state):
        return (state[0] == self.goal_state[0] and state[1] == self.goal_state[1])
    
    def actions(self, state):
        row = state[0]
        col = state[1]
        actions = []
        successor_states = []

        if ((row-1) >= 0 and grid[row-1][col] == ' '): #locating the previous row from the current state and moving upwards:
            actions.append("U") # U action results in (row-1, col)
            successor_states.append((row-1, col))

        if ((row+1 < len(grid) and grid[row+1][col] == ' ')): #locating the next row from the current position and moving downwards
            actions.append("D") # D actions results in (row+1, col)
            successor_states.append((row+1, col))

        if (col+1 < len(grid[0]) and grid[row][col +1] == ' '): # locating the current row and moving to the next column of from the current state:
            actions.append("R") # R actions results in (row, col+1)
            successor_states.append((row, col+1))

        if (col-1 >= 0 and grid[row][col-1] == ' '): # locating the current row and moving to the previous column from the current statee
            actions.append("L") # L actions results in (row, col-1)
            successor_states.append((row, col-1))
        return actions, successor_states

    def print_prob(self):
        print("__TwoDNavProb__")
        print("init_state =", self.init_state)
        print("goal_state = ", self.goal_state)
        print("grid = ")
        for i  in range(len(grid)):
            print("|", end = "")
            for j in range(len(grid[i])):
                if (self.init_state[0] == i and self.init_state[1] == j):
                    print("S|", end="")
                elif (self.goal_state[0] == i and self.goal_state[1] == j):
                    print("G|", end = "")
                else:
                    print(grid[i][j] + "|", end = "")
            print()
class Node:
    """ This class represents a node in the search tree """
    def __init__(self, state, parent = None, action = None, path_cost = 0):
        self.state = state
        self.parent = parent
        self.action = action
        self.path_cost = path_cost

    def __str__(self):
        mystr = "Node with state = " + str(self.state);
        if (self.parent != None):
            mystr += (", parent = " + str(self.parent.state) + ", action = " + str(self.action) + ", path_cost = " + str(self.path_cost))
        
        return mystr
    # The purpose of this method is to enable two nodes  on the
    # frontier to be considered equal to each other if they represent
    # the same state (regardless of if they have different parent nodes)

    def __eq__(self, other):
        return (isinstance(other, Node) and self.state == other.state)

    def solution_path(self):
        list_of_seq_of_actions = []
        list_of_seq_of_states = []
        while self != None:
            if self.parent:
                list_of_seq_of_actions.insert(0, self.action)
            list_of_seq_of_states.insert(0, self.state)
            self = self.parent

        pathCost = len(list_of_seq_of_actions)
        
        return (list_of_seq_of_actions, list_of_seq_of_states, pathCost )


        """To be implemented to calc the cost of the solution path leading to this node"""
        return None

def BreadthFirstSearch(problem):
    """
    A function to implement the bread-first search algorithm
    """
    print("About to do breadth-first search on problem: ", problem)
    node = Node(problem.init_state)
    if problem.goal_test(node.state):
        return node.solution_path()
    frontier = [node]
    explored = set()
    
    while(len(frontier) > 0):
        node = frontier.pop(0)
        explored.add(node.state)
        print("Popped: ", node)
        actions, successors = problem.actions(node.state)
        print("Generated successor states: ",successors)
        print("XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX")
        for i in range(len(actions)):
            child = Node(successors[i], node, actions[i], node.path_cost+1)
            if (child.state not in explored and child not in frontier):
                if (problem.goal_test(child.state)):
                    print("Found a solution! ", child)
                    return child.solution_path()
                frontier.append(child)
    return None

if __name__ == "__main__":
    print("Instantiating a TwoDNavProb")

    grid = [
            [' ', ' ', ' ', ' ', 'X'],
            [' ', 'X', ' ', ' ', ' '],
            [' ', ' ', 'X', 'X', ' '],
            [' ', 'X', ' ', ' ', ' ']
        ]
    myProb = TwoDNavProb((0,3),(3,2), grid)
    myProb.print_prob()
    print("XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX")
    solution = BreadthFirstSearch(myProb)
    print("BFS returned ", solution)
    print("XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX")
