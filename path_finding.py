from search_problem import SearchProblem
from world import World

# Assume that:
# State is a pair (x,y)
# World is as defined in the imported class World,
# so has limits given by x_lim and y_lim


class PathFinding(SearchProblem):
    def __init__(self, init, goal, world: World):
        self.actions = ['N', 'S', 'W', 'E']
        self.world = world
        super().__init__(init, goal, [(a, 1) for a in self.actions])

    def getSuccessors(self, state) -> set:
        succ = set()
        for a in self.actions:
            if a == 'S':
                next = (state[0], state[1]-1)
            elif a == 'N':
                next = (state[0], state[1]+1)
            elif a == 'W':
                next = (state[0]-1, state[1])
            elif a == 'E':
                next = (state[0]+1, state[1])
            if next not in self.world.walls and self.isInTheLimits(next):
                succ.add((a, next))
        return succ

    def isInTheLimits(self, state):
        return state[0] >= 0 and state[0] <= self.world.x_lim and state[1] >= 0 and state[1] <= self.world.y_lim

    def isGoal(self, state):
        return state[0] == self.goal[0] and state[1] == self.goal[1]
