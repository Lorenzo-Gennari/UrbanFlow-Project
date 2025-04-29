class World(object):
    def __init__(self, x_lim: int, y_lim: int, walls: set, ztl: set):
        self.x_lim = x_lim
        self.y_lim = y_lim
        self.walls = walls
        self.ztl = ztl

    def __str__(self):
        ret = ""
        for x in range(0, self.x_lim):
            for y in range(0, self.y_lim):
                if (x, y) in self.walls:
                    ret += "%"
                else:
                    ret += " "
            ret += "\n"
        return ret
