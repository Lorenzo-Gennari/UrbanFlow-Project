from search_problem import SearchProblem
from world import World

## Assume that:
# State is a pair (x,y)
# World is as defined in the imported class World, so has limits given by x_lim and y_lim

class PathFinding(SearchProblem):
    def __init__(self, init, goal, world: World):
        self.actions = ['N','S','W','E']
        self.world = world
        super().__init__(init, goal, [(a,1) for a in self.actions])    

    def getSuccessors(self, state) -> set:
        raise NotImplementedError("To be implemented")

    def isGoal(self, state):
        raise NotImplementedError("To be implemented")


