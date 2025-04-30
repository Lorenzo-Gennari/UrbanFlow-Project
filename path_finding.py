from search_problem import SearchProblem
from world import World

# Assume that:
# State is a pair (x,y)
# World is as defined in the imported class World,
# so has limits given by x_lim and y_lim


class PathFinding(SearchProblem):
    def __init__(self, init, goal, world: World):
        self.actions = ['W', 'E', 'S', 'N', 'NE', 'NW', 'SE', 'SW']
        #  self.actions = ['W', 'E', 'S', 'N']
        self.world = world
        super().__init__(init, goal, [(a, 1) for a in self.actions])

    def getSuccessors(self, state, truck) -> set:
        succ = []
        for a in self.actions:
            if a == 'S':
                x = state[0]
                y = state[1]-1
                next = (x, y)
            elif a == 'N':
                x = state[0]
                y = state[1]+1
                next = (x, y)
            elif a == 'W':
                x = state[0]-1
                y = state[1]
                next = (x, y)
            elif a == 'E':
                x = state[0]+1
                y = state[1]
                next = (x, y)
            elif a == 'NE':
                x = state[0]+1
                y = state[1]+1
                next = (x, y)
            elif a == 'NW':
                x = state[0]-1
                y = state[1]+1
                next = (x, y)
            elif a == 'SE':
                x = state[0]+1
                y = state[1]-1
                next = (x, y)
            elif a == 'SW':
                x = state[0]-1
                y = state[1]-1
                next = (x, y)
            if next not in self.world.walls and self.isInTheLimits(next):
                if truck.type == "diesel":
                    if next not in self.world.ztl:
                        succ.append((a, next))
                else:
                    succ.append((a, next))
        return succ

    def isInTheLimits(self, state):
        return state[0] >= 0 and state[0] <= self.world.x_lim and state[1] >= 0 and state[1] <= self.world.y_lim

    def isGoal(self, state):
        return state[0] == self.goal[0] and state[1] == self.goal[1]
