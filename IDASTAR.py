import math
from search_algorithm import SearchAlgorithm, Node
import heuristics as h


class AstarNode(Node):
    def __init__(self, state, parent=None, action=None, g=0, h=0) -> None:
        self.h = h
        super().__init__(state, parent, action, g)

    def __lt__(self, other):
        return self.g + self.h < other.g + other.h


class IDAStar(SearchAlgorithm):
    def __init__(self, heuristic=lambda x, y: 0, view=False, w=1) -> None:
        self.heuristic = heuristic
        self.w = w
        super().__init__(view)

    def solve(self, problem, draw, grid, he) -> list:
        start_node = AstarNode(
            problem.init,
            None,
            None,
            g=0,
            h=h.choose_heuristic(he, problem.init, problem.goal)
        )

        bound = start_node.h
        self.goal_node = None

        while True:
            visited = set()
            next_bound = self._search(
                start_node, bound, visited, problem, draw, grid, he)

            if self.goal_node:
                return self.extract_solution(self.goal_node)
            if next_bound == math.inf:
                return []
            bound = next_bound

    def _search(self, node, bound, visited, problem, draw, grid, he):
        if node.state in visited:
            return math.inf
        visited.add(node.state)

        f = node.g + self.w * node.h
        if f > bound:
            return f
        if problem.isGoal(node.state):
            self.goal_node = node
            return "found"

        min_threshold = math.inf
        states = problem.getSuccessors(node.state)

        states.sort(key=lambda x: h.choose_heuristic(he, x[1], problem.goal))

        for action, succ_state in states:
            child_h = h.choose_heuristic(he, succ_state, problem.goal)
            child_node = AstarNode(
                succ_state, node, action, node.g + 1, child_h)
            if child_node.state in visited or self.is_expanded(child_node.state):
                continue

            self.update_expanded(succ_state)
            grid[succ_state[0]][succ_state[1]].make_closed()

            result = self._search(
                child_node, bound, visited, problem, draw, grid, he)

            if result == "found":
                return "found"
            if result < min_threshold:
                min_threshold = result

            grid[succ_state[0]][succ_state[1]].make_open()

        draw()
        return min_threshold
