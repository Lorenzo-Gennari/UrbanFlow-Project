from search_algorithm import SearchAlgorithm
from queue import Queue
from search_algorithm import Node


class BrFS(SearchAlgorithm):
    """Breath First Search

    Args:
        Solver (_type_): This is an implementation for the Solver class
    """

    def solve(self, problem, win, grid, rows, width, func) -> list:
        node = Node(problem.init, None, None, 0)
        reached = set()
        reached.add(node.state)
        frontier = Queue()
        frontier.put(node)
        while not frontier.empty():
            node = frontier.get()
            states = problem.getSuccessors(node.state)
            for state in states:
                child_node = Node(state[1], node, state[0], 0)
                if child_node.state not in reached and child_node not in list(frontier.queue):
                    reached.add(child_node.state)
                    self.update_expanded(child_node.state)
                    grid[state[1][0]][state[1][1]].make_open()
                    if problem.isGoal(child_node.state):
                        print("goal node: ", child_node.state)
                        return self.extract_solution(child_node)
                    else:
                        frontier.put(child_node)
                        if node.parent is not None:
                            grid[node.state[0]][node.state[1]].make_closed()
                    func(win, grid, rows, width)
