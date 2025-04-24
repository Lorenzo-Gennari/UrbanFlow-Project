import math
from queue import LifoQueue
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

        bound = start_node.h  # Inizia con l'euristica come bound
        visited = set()  # Set per tenere traccia dei nodi visitati

        while True:
            frontier = LifoQueue()  # Crea la coda di esplorazione
            frontier.put(start_node)
            t = self._search(frontier, 0, bound, he,
                             problem, grid, visited, draw)
            if t == "found":
                return self.extract_solution(frontier.get())
            if t == math.inf:
                return []
            bound = t

    def _search(self, frontier, g, bound, he, problem, grid, visited, draw):
        node = frontier.get()
        f = g + h.choose_heuristic(he, node.state, problem.goal)

        if f > bound:  # Se il costo supera il bound, ritorna il valore di f
            return f
        if problem.isGoal(node.state):  # Se troviamo l'obiettivo, ritorna "found"
            frontier.put(node)
            return "found"

        visited.add(node.state)  # Aggiungi il nodo allo stato visitato
        min_threshold = math.inf
        states = problem.getSuccessors(node.state)

        for action, succ_state in states:
            if succ_state in visited:  # Se il successore è già stato visitato, saltalo
                continue

            # Calcola il valore dell'euristica per il successore
            child_h = h.choose_heuristic(he, succ_state, problem.goal)
            child_node = AstarNode(succ_state, node, action, g + 1, child_h)

            grid[succ_state[0]][succ_state[1]].make_closed()  # Nodo esplorato
            frontier.put(child_node)

            # Esplora il nodo ricorsivamente
            t = self._search(frontier, g + 1, bound, he,
                             problem, grid, visited, draw)
            if t == "found":  # Se troviamo l'obiettivo, ritorna "found"
                return "found"
            if t < min_threshold:  # Aggiorna il minimo threshold trovato
                min_threshold = t

            self.update_expanded(child_node.state)
            grid[succ_state[0]][succ_state[1]].make_open()
            draw()

        return min_threshold  # Ritorna il threshold minimo trovato
