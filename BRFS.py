from search_algorithm import SearchAlgorithm
from queue import Queue
from search_algorithm import Node


class BrFS(SearchAlgorithm):
    """Breath First Search

    Args:
        Solver (_type_): This is an implementation for the Solver class
    """

    def solve(self, problem) -> (list, set):
        node = Node(problem.init, None, None, 0)
        reached = set()
        reached.add(node.state)
        frontier = Queue()
        frontier.put(node)
        while not frontier.empty():
            node = frontier.get()
            reached.add(node.state)
            self.update_expanded(node.state)
            print("current node: ", node.state)
            states = problem.getSuccessors(node.state)
            for state in states:
                child_node = Node(state[1], node, state[0], 0)
                print("action: ", child_node.action)
                if child_node.state not in reached:
                    if problem.isGoal(child_node.state):
                        print("goal node: ", child_node.state)
                        return (self.extract_solution(child_node), reached)
                    else:
                        print("not goal node: ", child_node.state)
                        frontier.put(child_node)
