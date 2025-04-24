import math
from search_algorithm import SearchAlgorithm, Node
import heuristics as h


class AstarNode(Node):
    def __init__(self, state, parent=None, action=None, g=0, h=0) -> None:
        self.h = h
        super().__init__(state, parent, action, g)

    def f(self):
        return self.g + self.h


class IDAStar(SearchAlgorithm):
    def __init__(self, heuristic=lambda x, y: 0, view=False, w=1) -> None:
        self.heuristic = heuristic
        self.w = w
        self.heuristic_table = {}
        super().__init__(view)

    def solve(self, problem, draw, grid, he) -> list:
        # Metodo solve mantenuto invariato
        self._precompute_heuristics(problem, he, grid)
        start_node = AstarNode(
            problem.init,
            None,
            None,
            g=0,
            h=self.heuristic_table[problem.init]
        )

        bound = self.w * start_node.h
        self.goal_node = None
        self.visited_global = set()

        while True:
            self.visited_local = set()
            next_bound = self._search(start_node, bound, problem, draw, grid)

            if self.goal_node:
                return self.extract_solution(self.goal_node)
            if next_bound == math.inf:
                return []
            bound = next_bound

    def _precompute_heuristics(self, problem, he, grid):
        """Aggiunta penalità ostacoli vicini"""
        for x in range(len(grid)):
            for y in range(len(grid)):
                if not grid[x][y].is_barrier():
                    base_h = h.choose_heuristic(he, (x, y), problem.goal)
                    obstacle_penalty = self._obstacle_density(
                        (x, y), grid) * 0.2
                    self.heuristic_table[(x, y)] = base_h + obstacle_penalty

    def _search(self, node, bound, problem, draw, grid):
        """Aggiunta dead-end detection"""
        f = node.f()

        if f > bound or node.state in self.visited_local:
            return f

        if problem.isGoal(node.state):
            self.goal_node = node
            return "found"

        if self._dead_end_detection(node, grid):
            return math.inf

        self.visited_local.add(node.state)
        min_threshold = math.inf
        successors = self._get_optimized_successors(node, problem, grid)

        for action, succ_state in successors:
            grid[succ_state[0]][succ_state[1]].make_closed()
            draw()

            child_node = AstarNode(
                succ_state,
                node,
                action,
                node.g + 1,
                self.heuristic_table[succ_state]
            )

            result = self._search(child_node, bound, problem, draw, grid)

            if result == "found":
                return "found"
            if result < min_threshold:
                min_threshold = result

            grid[succ_state[0]][succ_state[1]].make_open()
            draw()

        self.visited_local.remove(node.state)
        return min_threshold

    def _get_optimized_successors(self, node, problem, grid):
        """Ordinamento ottimizzato con obstacle awareness"""
        current = node.state
        successors = problem.getSuccessors(node.state)

        successors.sort(key=lambda s: (
            self._direction_score(s[1], current, problem.goal),
            self._obstacle_density(s[1], grid)
        ))

        return successors

    def _direction_score(self, state, current, goal):
        """Calcola priorità direzionale normalizzata"""
        dx = state[0] - current[0]
        dy = state[1] - current[1]
        goal_dx = goal[0] - current[0]
        goal_dy = goal[1] - current[1]
        return math.hypot(dx - goal_dx, dy - goal_dy)

    def _obstacle_density(self, state, grid):
        """Calcola densità ostacoli in 3x3 grid"""
        count = 0
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                x, y = state[0] + dx, state[1] + dy
                if 0 <= x < len(grid) and 0 <= y < len(grid[0]):
                    count += 1 if grid[x][y].is_barrier() else 0
        return count

    def _dead_end_detection(self, node, grid):
        """Rilevamento rapido vicoli ciechi"""
        x, y = node.state
        neighbors = [
            (x+1, y), (x-1, y), (x, y+1), (x, y-1),
            (x+1, y+1), (x-1, y-1), (x+1, y-1), (x-1, y+1)
        ]
        return all(grid[n[0]][n[1]].is_barrier() for n in neighbors if 0 <= n[0] < len(grid) and 0 <= n[1] < len(grid[0]))
