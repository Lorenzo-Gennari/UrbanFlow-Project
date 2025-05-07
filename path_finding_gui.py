import time
import click
import pygame
import json
from path_finding import PathFinding
from ASTAR import AStar as ASTARPathFinder
from DFS import DFS as DFSPathFinder
from BRFS import BrFS as BRFSPathFinder
from IDASTAR import IDAStar as IDASTARPathFinder
import heuristics
from world import World

WIDTH = 1000
WIN = pygame.display.set_mode((WIDTH, WIDTH))
pygame.display.set_caption("Search Algorithm")
pygame.init()

BASE_IMAGE = pygame.image.load("warehouse.png")
BASE_SIZE = 3

RED_TRANS = (255, 0, 0, 128)
RED = (255, 0, 0)
GREEN = (0, 255, 0)
GREEN_TRANS = (0, 255, 0, 10)
BLUE = (0, 255, 0)
YELLOW = (255, 255, 0)
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
PURPLE = (128, 0, 128)
ORANGE = (255, 165, 0)
GREY = (128, 128, 128)
TURQUOISE = (64, 224, 208)
TRANSPARENT = (0, 0, 0, 0)


class Truck:
    def __init__(self, truck_type="electric"):
        self.type = truck_type
        self.image = None
        self.load_image()

    def load_image(self):
        if self.type == "diesel":
            self.image = pygame.image.load("truck-diesel.png")
        elif self.type == "electric":
            self.image = pygame.image.load("truck.png")
        return self.image


truck = Truck("electric")


class Spot:
    def __init__(self, row, col, width):
        self.row = row
        self.col = col
        self.x = row * width
        self.y = col * width
        self.color = WHITE
        self.width = width

        self.barrier = False
        self.closed = False
        self.open = False
        self.path = False
        self.start = False
        self.end = False

        self.rotation = -90

        self.ztl = False

    def get_pos(self):
        return self.row, self.col

    def is_closed(self):
        return self.closed

    def is_open(self):
        return self.open

    def is_barrier(self):
        return self.barrier

    def is_start(self):
        return self.start

    def is_end(self):
        return self.end

    def is_path(self):
        return self.path

    def is_ztl(self):
        return self.ztl

    def reset(self):
        self.color = WHITE
        self.barrier = False
        self.closed = False
        self.open = False
        self.path = False
        self.start = False
        self.end = False
        self.ztl = False

    def make_start(self):
        if not self.end:
            self.reset()
            self.start = True
            self.color = ORANGE

    def make_closed(self):
        if not self.end:
            self.reset()
            self.closed = True
            self.color = RED

    def make_open(self):
        if not self.end:
            self.reset()
            self.open = True
            self.color = GREEN

    def make_barrier(self):
        if not self.end:
            self.reset()
            self.barrier = True
            self.color = BLACK

    def make_ztl(self):
        if not self.end:
            self.reset()
            self.ztl = True
            self.color = RED

    def make_end(self):
        self.reset()
        self.end = True
        self.color = TURQUOISE

    def make_path(self):
        if not self.end:
            self.reset()
            self.path = True
            self.color = PURPLE

    def draw(self, win):
        cell_size = WIDTH // 50

        if self.is_barrier():
            s = pygame.Surface((self.width, self.width), pygame.SRCALPHA)
            s.fill(TRANSPARENT)
            win.blit(s, (self.x, self.y))

        elif self.is_ztl():
            s = pygame.Surface((self.width, self.width), pygame.SRCALPHA)
            s.fill(RED_TRANS)
            win.blit(s, (self.x, self.y))

        elif self.is_open():
            pygame.draw.rect(
                win, GREEN, (self.x, self.y, self.width, self.width))

        elif self.is_closed():
            pygame.draw.rect(
                win, RED, (self.x, self.y, self.width, self.width))

        elif self.is_path():
            pygame.draw.rect(
                win, PURPLE, (self.x, self.y, self.width, self.width))

        elif self.is_start():
            truck_pixel_size = 3 * cell_size
            scaled_truck = pygame.transform.scale(
                truck.load_image(), (truck_pixel_size, truck_pixel_size))
            scaled_truck = pygame.transform.rotate(
                scaled_truck, self.rotation)
            win.blit(scaled_truck, (self.x-cell_size, self.y-cell_size))

        elif self.is_end():
            base_pixel_size = BASE_SIZE * cell_size
            scaled_base = pygame.transform.scale(
                BASE_IMAGE, (base_pixel_size, base_pixel_size))
            win.blit(scaled_base, (self.x-cell_size, self.y-cell_size))

    def __str__(self):
        return "({},{})".format(self.row, self.col)


def make_grid(rows, width):
    grid = []
    gap = width // rows
    for i in range(rows):
        grid.append([])
        for j in range(width):
            spot = Spot(i, j, gap)
            grid[i].append(spot)

    return grid


def make_grid_from_file(filename, width):
    f = open(filename)

    data = json.load(f)

    rows = data['rows']
    grid = []
    gap = width // rows

    start = (data['start'][0], data['start'][1])
    end = (data['end'][0], data['end'][1])

    barrier = {(ele[0], ele[1]) for ele in data['barrier']}

    background_image = None
    if 'background' in data and data['background']:
        try:
            background_image = pygame.image.load(
                data['background'])
            background_image = pygame.transform.scale(
                background_image, (WIDTH-200, WIDTH-200))
        except pygame.error as e:
            print(f"Errore nel caricare l'immagine di background: {e}")

    ztl = set()
    if 'ztl' in data:
        ztl = {(i, j) for i, j in data['ztl']}

    for i in range(rows):
        grid.append([])
        for j in range(rows):
            spot = Spot(i, j, gap)
            if (i, j) in barrier:
                spot.make_barrier()
            elif (i, j) == start:
                spot.make_start()
                start = spot
            elif (i, j) == end:
                spot.make_end()
                end = spot
            elif (i, j) in ztl:
                spot.make_ztl()
            grid[i].append(spot)

    return grid, start, end, rows, barrier, background_image, ztl


def draw_grid(win, rows, width):
    gap = width // rows
    for i in range(rows):
        pygame.draw.line(win, GREY, (0, i * gap), (width, i * gap))
        for j in range(rows):
            pygame.draw.line(win, GREY, (j * gap, 0), (j * gap, width))


def draw(win, grid, rows, width, background=None):
    win.fill(WHITE)
    if background:
        win.blit(background, (0, 0))

    for row in grid:
        for spot in row:
            if not spot.is_start() and not spot.is_end():
                spot.draw(win)

    for row in grid:
        for spot in row:
            if spot.is_start():
                spot.draw(win)
            if spot.is_end():
                spot.draw(win)
            if spot.is_ztl():
                spot.draw(win)
            if spot.is_barrier():
                spot.draw(win)

    #  draw_grid(win, rows, width)
    save_map_button.show()
    manhattan.show()
    chebyshev.show()
    euclidean.show()
    blind.show()
    electric.show()
    diesel.show()
    map1.show()
    map2.show()
    map3.show()
    map4.show()
    map5.show()
    astar.show()
    idastar.show()
    breathfs.show()
    all.show()

    pygame.display.update()


def get_clicked_pos(pos, rows, width):
    gap = width // rows
    y, x = pos

    row = y // gap
    col = x // gap

    return row, col


def mark_spots(start, grid, plan):

    x = start.row
    y = start.col
    for a in plan:
        if a == 'N':
            y += 1
        elif a == 'S':
            y -= 1
        elif a == 'E':
            x += 1
        elif a == 'W':
            x -= 1
        elif a == 'NE':
            x += 1
            y += 1
        elif a == 'NW':
            x -= 1
            y += 1
        elif a == 'SE':
            x += 1
            y -= 1
        elif a == 'SW':
            x -= 1
            y -= 1
        grid[x][y].make_path()

        if not grid[x][y].is_end():  # Evita di cambiare la base
            grid[x][y].make_path()


def animate_truck(start, plan, grid, rows, background=None):
    for row in grid:
        for spot in row:
            if spot.is_open() or spot.is_closed():
                spot.reset()

    x, y = start.row, start.col

    draw_grid(WIN, rows, WIDTH-200)
    draw(WIN, grid, rows, WIDTH-200, background)

    '''
    for move, next_move in zip(plan, plan[1:]):
        pygame.time.delay(50)

        if not grid[x][y].is_end():
            grid[x][y].reset()

        if move == 'N':
            y += 1
            if next_move == 'W':
                grid[x][y].rotation = -235
            elif next_move == 'E':
                grid[x][y].rotation = +235
            else:
                grid[x][y].rotation = +180
        elif move == 'S':
            y -= 1
            if next_move == 'W':
                grid[x][y].rotation = +45
            elif next_move == 'E':
                grid[x][y].rotation = -45
        elif move == 'E':
            x += 1
            if next_move == 'N':
                grid[x][y].rotation = +235
            elif next_move == 'S':
                grid[x][y].rotation = -45
            else:
                grid[x][y].rotation = -90
        elif move == 'W':
            x -= 1
            if next_move == 'N':
                grid[x][y].rotation = -235
            elif next_move == 'S':
                grid[x][y].rotation = +45
            else:
                grid[x][y].rotation = +90

        grid[x][y].make_start()
    '''
    for move, next_move in zip(plan, plan[1:]):
        pygame.time.delay(50)

        if not grid[x][y].is_end():
            grid[x][y].reset()

        if move == 'N':
            y += 1
        elif move == 'S':
            y -= 1
        elif move == 'E':
            x += 1
        elif move == 'W':
            x -= 1
        elif move == 'NE':  # Diagonale Nord-Est
            x += 1
            y += 1
        elif move == 'NW':  # Diagonale Nord-Ovest
            x -= 1
            y += 1
        elif move == 'SE':  # Diagonale Sud-Est
            x += 1
            y -= 1
        elif move == 'SW':  # Diagonale Sud-Ovest
            x -= 1
            y -= 1

        grid[x][y].make_start()

        if background:
            WIN.blit(background, (0, 0))
        draw(WIN, grid, rows, WIDTH-200, background)

        pygame.display.update()

    pygame.display.update()


def mark_expanded(exp, grid):
    for e in exp:
        grid[e[0]][e[1]].make_closed()


def save_to_file(grid, start, end, filename="temp.json"):
    barrier = list()
    ztl = list()
    for x in grid:
        for spot in x:
            if spot.is_barrier():
                barrier.append((spot.row, spot.col))
            if spot.is_ztl():
                ztl.append((spot.row, spot.col))
    res = {"rows": len(grid), "start": (start.row, start.col),
           "end": (end.row, end.col), "barrier": barrier, "ztl": ztl}
    data = json.dumps(res, indent=4)
    with open(filename, "w") as data_file:
        data_file.write(data)


def load_from_file(filename):
    f = open(filename)

    data = json.load(f)

    rows = data['rows']
    grid = []
    gap = width // rows

    start = (data['start'][0], data['start'][1])
    end = (data['end'][0], data['end'][1])

    barrier = {(ele[0], ele[1]) for ele in data['barrier']}

    for i in range(rows):
        grid.append([])
        for j in range(rows):
            spot = Spot(i, j, gap)
            if (i, j) in barrier:
                spot.make_barrier()
            elif (i, j) == start:
                spot.make_start()
                start = spot
            elif (i, j) == end:
                spot.make_end()
                end = spot
            grid[i].append(spot)

    return grid, start, end, rows, barrier


selected_heuristic = "m"


def make_plan(p, draw, win, grid, rows, width, search_algorithm, background, he):
    if isinstance(search_algorithm, ASTARPathFinder) or isinstance(search_algorithm, IDASTARPathFinder):
        plan = search_algorithm.solve(p, lambda: draw(
            win, grid, rows, width, background), grid, he, truck)
    else:
        plan = search_algorithm.solve(
            p, lambda: draw(win, grid, rows, width, background), grid, truck)

    return plan


def choose_plan(plans):
    min = WIDTH*WIDTH
    min_plan = None
    min_type = None
    for plan, ty in plans:
        if len(plan) < min:
            min = len(plan)
            min_plan = plan
            min_type = ty
    return min_plan, min_type


def update_selected_heuristic(event, search_algorithm):
    global selected_heuristic

    heuristic_selected = manhattan.click(event, search_algorithm, "m")
    if heuristic_selected:
        selected_heuristic = heuristic_selected
        chebyshev.change_text("chebyshev", bg="navy")
        euclidean.change_text("eucledian", bg="navy")
        blind.change_text("blind", bg="navy")

    heuristic_selected = chebyshev.click(event, search_algorithm, "c")
    if heuristic_selected:
        selected_heuristic = heuristic_selected
        manhattan.change_text("manhattan", bg="navy")
        euclidean.change_text("eucledian", bg="navy")
        blind.change_text("blind", bg="navy")

    heuristic_selected = euclidean.click(event, search_algorithm, "e")
    if heuristic_selected:
        selected_heuristic = heuristic_selected
        manhattan.change_text("manhattan", bg="navy")
        chebyshev.change_text("chebyshev", bg="navy")
        blind.change_text("blind", bg="navy")

    heuristic_selected = blind.click(event, search_algorithm, "b")
    if heuristic_selected:
        selected_heuristic = heuristic_selected
        manhattan.change_text("manhattan", bg="navy")
        chebyshev.change_text("chebyshev", bg="navy")
        euclidean.change_text("eucledian", bg="navy")


class Button:
    """Create a button, then blit the surface in the while loop"""

    def __init__(self, text,  pos, font, bg="black", feedback=""):
        self.x, self.y = pos
        self.font = pygame.font.SysFont("Arial", font)
        if feedback == "":
            self.feedback = "text"
        else:
            self.feedback = feedback
        self.change_text(text, bg)
        self.name = text

    def change_text(self, text, bg="black"):
        self.text = self.font.render(text, 1, pygame.Color("White"))
        self.size = self.text.get_size()
        self.surface = pygame.Surface(self.size)
        self.surface.fill(bg)
        self.surface.blit(self.text, (0, 0))
        self.rect = pygame.Rect(self.x, self.y, self.size[0], self.size[1])

    def show(self):
        WIN.blit(self.surface, (self.x, self.y))

    def click_save(self, event, grid, start, end):
        x, y = pygame.mouse.get_pos()
        if event.type == pygame.MOUSEBUTTONDOWN:
            if pygame.mouse.get_pressed()[0]:
                if self.rect.collidepoint(x, y):
                    if grid is not None and start is not None and end is not None:
                        save_to_file(grid, start, end, "tempmap.json")
                        self.change_text(self.feedback, bg="red")

    def click(self, event, search_algorithm, heuristic_type):
        x, y = pygame.mouse.get_pos()
        if event.type == pygame.MOUSEBUTTONDOWN:
            if pygame.mouse.get_pressed()[0]:
                if self.rect.collidepoint(x, y):
                    search_algorithm.heuristic = heuristic_type
                    self.change_text(self.feedback, bg="red")
                    return heuristic_type
        return None

    def click_truck(self, event, truck):
        x, y = pygame.mouse.get_pos()
        if event.type == pygame.MOUSEBUTTONDOWN:
            if pygame.mouse.get_pressed()[0]:
                if self.rect.collidepoint(x, y):
                    if self.name == "electric":
                        truck.type = "electric"
                        self.change_text(self.feedback, bg="green")
                        diesel.change_text("diesel", bg="navy")
                    elif self.name == "diesel":
                        truck.type = "diesel"
                        self.change_text(self.feedback, bg="green")
                        electric.change_text("electric", bg="navy")

    def click_map(self, event):
        x, y = pygame.mouse.get_pos()
        if event.type == pygame.MOUSEBUTTONDOWN:
            if pygame.mouse.get_pressed()[0]:
                if self.rect.collidepoint(x, y):
                    if self.name == "map 1":
                        self.change_text(self.feedback, bg="purple")
                        map2.change_text("map 2", bg="navy")
                        map3.change_text("map 3", bg="navy")
                        map4.change_text("map 4", bg="navy")
                        map5.change_text("map 5", bg="navy")
                        return make_grid_from_file("maps/mappa-città.json", WIDTH-200)
                    elif self.name == "map 2":
                        self.change_text(self.feedback, bg="purple")
                        map1.change_text("map 1", bg="navy")
                        map3.change_text("map 3", bg="navy")
                        map4.change_text("map 4", bg="navy")
                        map5.change_text("map 5", bg="navy")
                        return make_grid_from_file("maps/mappa-grande-2.json", WIDTH-200)
                    elif self.name == "map 3":
                        self.change_text(self.feedback, bg="purple")
                        map1.change_text("map 1", bg="navy")
                        map2.change_text("map 2", bg="navy")
                        map4.change_text("map 4", bg="navy")
                        map5.change_text("map 5", bg="navy")
                        return make_grid_from_file("maps/mappa-grande-3.json", WIDTH-200)
                    elif self.name == "map 4":
                        self.change_text(self.feedback, bg="purple")
                        map1.change_text("map 1", bg="navy")
                        map2.change_text("map 2", bg="navy")
                        map3.change_text("map 3", bg="navy")
                        map5.change_text("map 5", bg="navy")
                        return make_grid_from_file("maps/mappa-grande-4.json", WIDTH-200)
                    elif self.name == "map 5":
                        self.change_text(self.feedback, bg="purple")
                        map1.change_text("map 1", bg="navy")
                        map2.change_text("map 2", bg="navy")
                        map3.change_text("map 3", bg="navy")
                        map4.change_text("map 4", bg="navy")
                        return make_grid_from_file("maps/mappa-grande-5.json", WIDTH-200)

    def click_algorithm(self, event):
        x, y = pygame.mouse.get_pos()
        if event.type == pygame.MOUSEBUTTONDOWN:
            if pygame.mouse.get_pressed()[0]:
                if self.rect.collidepoint(x, y):
                    if self.name == "A*":
                        self.change_text(self.feedback, bg="orange")
                        idastar.change_text("IDA*", bg="navy")
                        breathfs.change_text("BRFS", bg="navy")
                        return ASTARPathFinder(heuristics.manhattan, True)
                    elif self.name == "IDA*":
                        self.change_text(self.feedback, bg="orange")
                        astar.change_text("A*", bg="navy")
                        breathfs.change_text("BRFS", bg="navy")
                        return IDASTARPathFinder(True)
                    elif self.name == "BRFS":
                        self.change_text(self.feedback, bg="orange")
                        astar.change_text("A*", bg="navy")
                        idastar.change_text("IDA*", bg="navy")
                        return BRFSPathFinder(True)

    def click_all(self, event, start, end, world, draw, win, grid, rows, width, background):
        x, y = pygame.mouse.get_pos()
        if event.type == pygame.MOUSEBUTTONDOWN:
            if pygame.mouse.get_pressed()[0]:
                if self.rect.collidepoint(x, y):
                    self.change_text(self.feedback, bg="brown")
                    heurs = ["m"]
                    trucks = ["diesel", "electric"]
                    algorithms = [ASTARPathFinder(
                        heuristics.manhattan, True), IDASTARPathFinder(True)]
                    p = PathFinding((start.row, start.col),
                                    (end.row, end.col), world)
                    plans = []
                    for tru in trucks:
                        truck.type = tru
                        for he in heurs:
                            selected_heuristic = he
                            for alg in algorithms:
                                plan = make_plan(p, draw, win, grid, rows,
                                                 width, alg, background, selected_heuristic), truck.type
                                plans.append(plan)
                                print(plan)
                        last_alg = BRFSPathFinder(True)
                        last = make_plan(p, draw, win, grid, rows,
                                         width, last_alg, background, selected_heuristic), truck.type
                        plans.append(last)
                        print(last)
                    best, truck.type = choose_plan(plans)
                    mark_spots(start, grid, best)
                    animate_truck(start, best, grid, rows, background)
                    self.change_text("DO ALL", bg="navy")
                    return plans


map1 = Button(
    "map 1",
    (WIDTH-200, 15),
    font=20,
    bg="navy",
    feedback="map 1 <=")

map2 = Button(
    "map 2",
    (WIDTH-200, 40),
    font=20,
    bg="navy",
    feedback="map 2 <=")

map3 = Button(
    "map 3",
    (WIDTH-200, 65),
    font=20,
    bg="navy",
    feedback="map 3 <=")

map4 = Button(
    "map 4",
    (WIDTH-200, 90),
    font=20,
    bg="navy",
    feedback="map 4 <=")

map5 = Button(
    "map 5",
    (WIDTH-200, 115),
    font=20,
    bg="navy",
    feedback="map 5 <=")

astar = Button(
    "A*",
    (WIDTH-200, 150),
    font=20,
    bg="navy",
    feedback="A* <=")

idastar = Button(
    "IDA*",
    (WIDTH-200, 175),
    font=20,
    bg="navy",
    feedback="IDA* <=")

breathfs = Button(
    "BRFS",
    (WIDTH-200, 200),
    font=20,
    bg="navy",
    feedback="BRFS <=")

manhattan = Button(
    "manhattan",
    (WIDTH-200, 250),
    font=20,
    bg="navy",
    feedback="manhattan <=")

chebyshev = Button(
    "chebyshev",
    (WIDTH-200, 275),
    font=20,
    bg="navy",
    feedback="chebyshev <=")

euclidean = Button(
    "eucledian",
    (WIDTH-200, 300),
    font=20,
    bg="navy",
    feedback="eucledian <=")

blind = Button(
    "blind",
    (WIDTH-200, 325),
    font=20,
    bg="navy",
    feedback="blind <=")

electric = Button(
    "electric",
    (WIDTH-200, 375),
    font=20,
    bg="navy",
    feedback="electric <=")

diesel = Button(
    "diesel",
    (WIDTH-200, 400),
    font=20,
    bg="navy",
    feedback="diesel <=")


all = Button(
    "DO ALL",
    (WIDTH-200, 450),
    font=20,
    bg="navy",
    feedback="DO ALL <=")


save_map_button = Button(
    "Save Map",
    (WIDTH-200, 700),
    font=20,
    bg="navy",
    feedback="Saved")


clock = pygame.time.Clock()


@click.command()
@click.option('-w', '--width', default=WIDTH-200, help="Width of the Windows")
@click.option('-r', '--rows', default=50, help="Number of rows/columns in the map")
@click.option('-s', '--search_algorithm', default="ASTAR", help="Search algorithm to be used")
@click.option('-f', '--filename', default=None, help="Initialize map with data from file")
def main(width, rows, search_algorithm, filename=None):
    win = WIN
    start = None
    end = None
    background = None
    ztl = set()
    ROWS = rows
    if search_algorithm == 'DFS':
        search_algorithm = DFSPathFinder(True)
    elif search_algorithm == 'ASTAR':
        search_algorithm = ASTARPathFinder(heuristics.manhattan, True)
    elif search_algorithm == 'ASTARW4':
        search_algorithm = ASTARPathFinder(heuristics.manhattan, True, w=4)
    elif search_algorithm == 'BRFS':
        search_algorithm = BRFSPathFinder(True)
    elif search_algorithm == 'IDASTAR':
        search_algorithm = IDASTARPathFinder(True)
    if filename is not None:

        grid, start, end, rows, wall, background = make_grid_from_file(
            filename, width)

        for i in list(wall):
            grid[i[0]][i[1]].make_barrier()
    else:
        grid = make_grid(rows, width)
        wall = set()
    run = True

    while run:
        draw(win, grid, rows, width, background)

        for event in pygame.event.get():
            pygame.display.update()
            if event.type == pygame.QUIT:
                run = False
            save_map_button.click_save(event, grid, start, end)
            update_selected_heuristic(event, search_algorithm)
            electric.click_truck(event, truck)
            diesel.click_truck(event, truck)
            if map1.click_map(event) is not None:
                grid, start, end, rows, wall, background, ztl = map1.click_map(
                    event)
            if map2.click_map(event) is not None:
                grid, start, end, rows, wall, background, ztl = map2.click_map(
                    event)
            if map3.click_map(event) is not None:
                grid, start, end, rows, wall, background, ztl = map3.click_map(
                    event)
            if map4.click_map(event) is not None:
                grid, start, end, rows, wall, background, ztl = map4.click_map(
                    event)
            if map5.click_map(event) is not None:
                grid, start, end, rows, wall, background, ztl = map5.click_map(
                    event)
            if astar.click_algorithm(event) is not None:
                search_algorithm = astar.click_algorithm(event)
            if idastar.click_algorithm(event) is not None:
                search_algorithm = idastar.click_algorithm(event)
            if breathfs.click_algorithm(event) is not None:
                search_algorithm = breathfs.click_algorithm(event)
            world = World(rows-1, rows-1, wall, ztl)
            all.click_all(event, start, end, world,
                          draw, win, grid, rows, width, background)

            if pygame.mouse.get_pressed()[0]:  # LEFT
                pos = pygame.mouse.get_pos()
                if pos[0] < width and pos[1] < width:
                    row, col = get_clicked_pos(pos, rows, width)
                    spot = grid[row][col]
                    if not start and spot != end:
                        start = spot
                        start.make_start()

                    elif not end and spot != start:
                        end = spot
                        end.make_end()

                    elif spot != end and spot != start:
                        # spot.make_barrier()
                        # wall.add((row, col))
                        spot.make_ztl()
                        ztl.add((row, col))

            elif pygame.mouse.get_pressed()[2]:  # RIGHT
                pos = pygame.mouse.get_pos()
                if pos[0] < width and pos[1] < width:
                    row, col = get_clicked_pos(pos, rows, width)
                    spot = grid[row][col]
                    spot.reset()
                    if spot == start:
                        start = None
                    elif spot == end:
                        end = None
                    elif spot.is_barrier():
                        wall.remove((row, col))

            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_SPACE and start and end:
                    world = World(rows-1, rows-1, wall, ztl)
                    p = PathFinding((start.row, start.col),
                                    (end.row, end.col), world)
                    now = time.time()
                    plan = make_plan(p, draw, win, grid, rows,
                                     width, search_algorithm, background, selected_heuristic)
                    now = time.time() - now
                    print("Number of Expansion: {} in {} seconds".format(
                        search_algorithm.expanded, now))
                    if plan is not None:
                        print(plan)
                        print("Cost of the plan is: {}".format(len(plan)))
                        mark_spots(start, grid, plan)
                        animate_truck(start, plan, grid, rows, background)
                    draw(win, grid, rows, width, background)
                if event.key == pygame.K_c:
                    start = None
                    end = None
                    grid = make_grid(rows, width)
                    wall = set()

    pygame.quit()


if __name__ == '__main__':
    main()
