from search_algorithm import SearchAlgorithm
from queue import LifoQueue
from search_algorithm import Node


class DFS(SearchAlgorithm):
    """Depth First Search

    Args:
        Solver (_type_): This is an implementation for the Solver class
    """

    def solve(self, problem, draw, grid, truck) -> list:
        node = Node(problem.init, None, None, 0)
        reached = set()
        reached.add(node.state)
        frontier = LifoQueue()
        frontier.put(node)
        while not frontier.empty():
            node = frontier.get()
            self.update_expanded(node.state)
            if problem.isGoal(node.state):
                print("goal node: ", node.state)
                return self.extract_solution(node)
            states = problem.getSuccessors(node.state, truck)
            for state in states:
                child_node = Node(state[1], node, state[0], 0)
                if child_node.state not in reached:
                    reached.add(child_node.state)
                    grid[state[1][0]][state[1][1]].make_open()
                    frontier.put(child_node)
            if node.parent is not None:
                grid[node.state[0]][node.state[1]].make_closed()
            draw()
