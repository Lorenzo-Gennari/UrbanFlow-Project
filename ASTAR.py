from search_algorithm import SearchAlgorithm
from queue import PriorityQueue
from search_algorithm import Node
import heuristics as h


class AstarNode(Node):
    def __init__(self, state, parent=None, action=None, g=0, h=0) -> None:
        self.h = h
        super().__init__(state, parent, action, g)

    def __lt__(self, other):
        return self.g + self.h < other.g + other.h


class AStar(SearchAlgorithm):
    """AStar First Search

    Args:
        Solver (_type_): This is an implementation for the Solver class
    """

    def __init__(self, heuristic=lambda x, y: 0, view=False, w=1) -> None:
        self.heuristic = heuristic
        self.w = w
        super().__init__(view)

    def solve(self, problem, draw, grid, he) -> list:
        print(he)
        frontier = PriorityQueue()
        reached = set()
        node = AstarNode(problem.init, None, None, 0,
                         h.choose_heuristic(he, problem.init, problem.goal))
        frontier.put(node)
        reached.add(node.state)
        while not frontier.empty():
            node = frontier.get()
            states = problem.getSuccessors(node.state)
            for state in states:
                child_node = AstarNode(
                    state[1], node, state[0], 0, h.choose_heuristic(he, state[1], problem.goal))
                if child_node.state not in reached:
                    self.update_expanded(child_node.state)
                    grid[state[1][0]][state[1][1]].make_open()
                    reached.add(child_node.state)
                    if problem.isGoal(child_node.state):
                        print("goal node: ", child_node.state)
                        return self.extract_solution(child_node)
                    else:
                        frontier.put(child_node)
            if node.parent is not None:
                grid[node.state[0]][node.state[1]].make_closed()
        draw()
