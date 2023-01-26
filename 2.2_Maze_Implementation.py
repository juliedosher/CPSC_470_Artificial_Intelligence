#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
@author: szczurpi

This program implements a search algorithm for solving a grid maze
It allows moves in 4 directions (1 point cost for each move)
"""

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
            [0, 0, 1, 1, 1],
            [1, 0, 0, 0, 0],
            [1, 1, 0, 1, 0],
            [1, 0, 0, 1, 0],
            [1, 0, 0, 1, 0],
            [1, 0, 1, 1, 0],
            [1, 0, 0, 0, 0],
            [0, 0, 1, 1, 1]], dtype=np.int32)
    
    maze[END] = EXIT

    def __init__(self, conf=START, g=0, pred_state=None, pred_action=None):
        """ Initializes the state with information passed from the arguments """
        self.pos = conf      # Configuration of the state - current coordinates
        self.gcost = g         # Path cost
        self.pred = pred_state  # Predecesor state
        self.action_from_pred = pred_action  # Action from predecesor state to current state
		### TODO - heuristic value for A*/greedy search ###
    
    def __hash__(self):
        """ Returns a hash code so that it can be stored in a set data structure """
        return self.pos.__hash__()
    
    def __eq__(self, other):
        """ Checks for equality of states by positions only """
        ### TODO ###
    
    def __lt__(self, other):
        """ Allows for ordering the states by the path (g) cost """
		### TODO ###
		
    def __str__(self):
        """ Returns the maze representation of the state """
        a = np.array(self.maze)
        a[self.pos] = 4
        return np.str(a)

    def is_goal(self):
        """ Returns true if current position is same as the exit position """
        return self.pos==MazeState().END
		
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
        print(self)
    
    def can_move(self, move):
        """ Returns true if agent can move in the given direction """
        ### TODO ###

                    
    def gen_next_state(self, move):
        """ Generates a new MazeState object by taking move from current state """
		### TODO ###

            
# Display the heading info
print('Artificial Intelligence')
print('Search algorithm implementation for a grid maze')
print('SEMESTER: Spring 2022')
print('NAME: Julie Dosher')
print()


# load start state onto frontier priority queue
frontier = queue.PriorityQueue()
start_state = MazeState()
frontier.put(start_state)

# Keep a closed set of states to which optimal path was already found
closed_set = set()

num_states = 0
while not frontier.empty():
    ### TODO ###
    h=0
                    
print('\nNumber of states visited =',num_states)
