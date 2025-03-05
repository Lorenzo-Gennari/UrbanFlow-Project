from search_algorithm import SearchAlgorithm
from queue import PriorityQueue
from search_algorithm import Node

class AstarNode(Node):
    def __init__(self, state, parent = None, action = None, g = 0, h = 0) -> None:
        self.h = h
        super().__init__(state, parent, action, g)
        
    def __lt__(self, other):
        return self.g + self.h < other.g + other.h 
    
class AStar(SearchAlgorithm):
    """AStar First Search

    Args:
        Solver (_type_): This is an implementation for the Solver class
    """
    def __init__(self, heuristic = lambda x,y : 0, view = False, w = 1) -> None:
        self.heuristic = heuristic
        self.w = w
        super().__init__(view)

    def solve(self, problem) -> list:
        raise NotImplementedError("To be implemented")

