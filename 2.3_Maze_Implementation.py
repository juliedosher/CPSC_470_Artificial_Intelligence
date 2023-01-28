import numpy as np
import queue # Needed for frontier queue
from heapq import heapify

class MazeState():
    """ Stores information about each visited state within the search """
    
    # Define constants
    SPACE = 0
    WALL = 1
    EXIT = 2
    START = (1,1)
    END = (9,1)
    maze = np.array([
            [0, 1, 1, 0, 1],
            [0, 0, 0, 0, 1],
            [0, 0, 1, 0, 1],
            [0, 1, 0, 0, 0],
            [0, 0, 0, 1, 0],
            [1, 1, 0, 1, 0],
            [0, 0, 0, 1, 0],
            [0, 0, 1, 1, 0],
            [0, 1, 0, 0, 0],
            [0, 0, 0, 1, 1]], dtype=np.int32)
    
    maze[END] = EXIT

    def __init__(self, conf=START, g=0, pred_state=None, pred_action=None):
        """ Initializes the state with information passed from the arguments """
        self.pos = conf      # Configuration of the state - current coordinates
        self.gcost = g         # Path cost
        self.hcost = self._get_heuristic_cost()
        self.fcost = self.hcost + self.gcost
        self.pred = pred_state  # Predecesor state
        self.action_from_pred = pred_action  # Action from predecesor state to current state
		
    
    def _get_heuristic_cost(self):
        x_dist = abs(self.pos[0] - MazeState.END[0])
        y_dist = abs(self.pos[1] - MazeState.END[1])
        return x_dist + y_dist
        #return np.sqrt((self.pos[0] - MazeState.END[0])**2 + (self.pos[1] - MazeState.END[1])**2)

    def __hash__(self):
        """ Returns a hash code so that it can be stored in a set data structure """
        return self.pos.__hash__()
    
    def __eq__(self, other):
        """ Checks for equality of states by positions only """
        return self.pos == other.pos
    
    def __lt__(self, other):
        """ Allows for ordering the states by the path (g) cost """
        return self.fcost < other.fcost
		
    def __str__(self):
        """ Returns the maze representation of the state """
        a = np.array(self.maze)
        a[self.pos] = 4
        return str(a)

    def is_goal(self):
        """ Returns true if current position is same as the exit position """
        return self.maze[self.pos] == MazeState.EXIT
		
    move_num = 0 # Used by show_path() to count moves in the solution path
    def show_path(self):
        """ Recursively outputs the list of moves and states along path """
        if self.pred is not None:
            self.pred.show_path()
        
        if MazeState.move_num==0:
            print('START')
        else:
            print('Move',MazeState.move_num, 'ACTION:', self.action_from_pred)
        MazeState.move_num = MazeState.move_num + 1
        #print(self)
        self.maze[self.pos] = 4
    
    def get_new_pos(self, move):
        if move=='up':
            new_pos = (self.pos[0]-1, self.pos[1])
        elif move=='down':
            new_pos = (self.pos[0]+1, self.pos[1])
        elif move=='left':
            new_pos = (self.pos[0], self.pos[1]-1)
        elif move=='right':
            new_pos = (self.pos[0], self.pos[1]+1)
        else:
            raise('wrong new position')
        return new_pos
    
    def can_move(self, move):
        """ Returns true if agent can move in the given direction """
        new_pos = self.get_new_pos(move)
        if new_pos[0]<0 or new_pos[0]>=self.maze.shape[0] or new_pos[1]<0 or new_pos[1]>=self.maze.shape[1]:
            return False
        else:
            return self.maze[new_pos]==MazeState.SPACE or self.maze[new_pos]==MazeState.EXIT
                    
    def gen_next_state(self, move):
        """ Generates a new MazeState object by taking move from current state """
        new_pos = self.get_new_pos(move)
        return MazeState(new_pos, self.gcost+1, self, move)
    
    def check_if_visited(self, set_of_states):
        has_visited = False
        
        for maze in set_of_states:
            if self.__eq__(maze):
                has_visited = True
                
        return has_visited

            
# Display the heading info
print('Artificial Intelligence')
print('Search algorithm implementation for a grid maze')
print('SEMESTER: Spring 2022')
print('NAME: Julie Dosher')
print()

# load start state onto frontier priority queue
frontier = queue.PriorityQueue()
start_state = MazeState()
frontier.put((1, start_state))
# Keep a closed set of states to which optimal path was already found
closed_set = set()

print("A* Search:")
num_states = 0
while not frontier.empty():
    curr_maze = frontier.get()[1]
    closed_set.add(curr_maze)
    
    for direction in ["right", "left", "up", "down"]:
        if curr_maze.can_move(direction):
            attempt = curr_maze.gen_next_state(direction)
            if(attempt.check_if_visited(closed_set)) == False:
                fcost = attempt.fcost
                frontier.put((fcost, attempt))
                num_states = num_states+1
        
    if curr_maze.is_goal():
        frontier.empty()
        path_cost = curr_maze.gcost

print('Number of states visited =',num_states)
print('Path cost = ',path_cost)


# reset the frontier, closed set, MazeState, etc. #######
frontier.empty()
start_state = MazeState()
frontier.put((1, start_state))
closed_set.clear()
num_states = 0
path_cost = 0

print("\nGreedy Search:")
while not frontier.empty():
    curr_maze = frontier.get()[1]
    closed_set.add(curr_maze)
    
    for direction in ["right", "left", "up", "down"]:
        if curr_maze.can_move(direction):
            attempt = curr_maze.gen_next_state(direction)
            if(attempt.check_if_visited(closed_set)) == False:
                hcost = attempt.hcost
                frontier.put((hcost, attempt))
                num_states = num_states+1
        
    if curr_maze.is_goal():
        frontier.empty()
        path_cost = curr_maze.gcost

print('Number of states visited =',num_states)
print('Path cost = ',path_cost)


# reset the frontier, closed set, MazeState, etc. #######
frontier.empty()
start_state = MazeState()
frontier.put((1, start_state))
closed_set.clear()
num_states = 0
path_cost = 0

print("\nUniform-Cost Search:")
while not frontier.empty():
    curr_maze = frontier.get()[1]
    closed_set.add(curr_maze)
    
    for direction in ["right", "left", "up", "down"]:
        if curr_maze.can_move(direction):
            attempt = curr_maze.gen_next_state(direction)
            if(attempt.check_if_visited(closed_set)) == False:
                gcost = attempt.gcost
                frontier.put((gcost, attempt))
                num_states = num_states+1
        
    if curr_maze.is_goal():
        frontier.empty()
        path_cost = curr_maze.gcost

print('Number of states visited =',num_states)
print('Path cost = ',path_cost)
