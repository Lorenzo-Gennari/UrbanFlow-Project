import math
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

BACKGROUND_IMAGE = pygame.image.load("mappa-colored.png")
BACKGROUND_IMAGE = pygame.transform.scale(
    BACKGROUND_IMAGE, (WIDTH-200, WIDTH-200))

BASE_IMAGE = pygame.image.load("warehouse.png")
BASE_SIZE = 3

TRUCK_ICON = pygame.image.load("truck.png")
TRUCK_SIZE = 3

RADAR_IMAGE = pygame.image.load("radar.png")
RADAR_SIZE = 3
RADIUS = 4


RED_TRANS = (255, 0, 0, 128)
RED = (255, 0, 0)
GREEN = (0, 255, 0)
GREEN_TRANS = (0, 255, 0, 128)
BLUE = (0, 255, 0)
YELLOW = (255, 255, 0)
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
PURPLE = (128, 0, 128)
ORANGE = (255, 165, 0)
GREY = (128, 128, 128)
TURQUOISE = (64, 224, 208)
TRANSPARENT = (0, 0, 0, 128)


class Spot:
    def __init__(self, row, col, width):
        self.row = row
        self.col = col
        self.x = row * width
        self.y = col * width
        self.color = WHITE  # Usa il colore solo per il rendering
        self.width = width

        # Nuovi stati booleani
        self.barrier = False
        self.closed = False
        self.open = False
        self.path = False
        self.start = False
        self.end = False

        # Nuovo nuovo stato
        self.rotation = 0

        # Nuovo nuovo nuovo stato
        self.area = False

    def get_pos(self):
        return self.row, self.col

    # Metodi corretti che controllano lo stato invece del colore
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

    def is_area(self):
        return self.area

    # Metodi per impostare gli stati
    def reset(self):
        self.color = WHITE
        self.barrier = False
        self.closed = False
        self.open = False
        self.path = False
        self.start = False
        self.end = False
        self.area = False

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

    def make_area(self):
        if not self.end:
            self.reset()
            self.area = True
            self.color = RED_TRANS

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
        cell_size = WIDTH // 50  # Dimensione della cella

        # Se è un ostacolo (barriera)
        if self.is_barrier():
            s = pygame.Surface((self.width, self.width), pygame.SRCALPHA)
            s.fill(GREEN_TRANS)
            win.blit(s, (self.x, self.y))

        elif self.is_area():
            s = pygame.Surface((self.width, self.width), pygame.SRCALPHA)
            s.fill(RED_TRANS)
            win.blit(s, (self.x, self.y))

        # Se è un nodo aperto (verde)
        elif self.is_open():
            pygame.draw.rect(
                win, GREEN, (self.x, self.y, self.width, self.width))

        # Se è un nodo chiuso (rosso)
        elif self.is_closed():
            pygame.draw.rect(
                win, RED, (self.x, self.y, self.width, self.width))

        # Se è parte del percorso trovato (viola)
        elif self.is_path():
            pygame.draw.rect(
                win, PURPLE, (self.x, self.y, self.width, self.width))

        # Se è il punto di partenza (aereo)
        elif self.is_start():
            airplane_pixel_size = TRUCK_SIZE * cell_size
            scaled_airplane = pygame.transform.scale(
                TRUCK_ICON, (airplane_pixel_size, airplane_pixel_size))
            scaled_airplane = pygame.transform.rotate(
                scaled_airplane, self.rotation)
            win.blit(scaled_airplane, (self.x-cell_size, self.y-cell_size))

        # Se è la destinazione (base militare)
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


def draw_grid(win, rows, width):
    gap = width // rows
    for i in range(rows):
        pygame.draw.line(win, GREY, (0, i * gap), (width, i * gap))
        for j in range(rows):
            pygame.draw.line(win, GREY, (j * gap, 0), (j * gap, width))


def draw(win, grid, rows, width):
    win.blit(BACKGROUND_IMAGE, (0, 0))  # Disegna lo sfondo prima di tutto

    # Prima, disegna tutti gli spot normali (barriere, percorsi, nodi)
    for row in grid:
        for spot in row:
            if not spot.is_start() and not spot.is_end():
                spot.draw(win)

    for row in grid:
        for spot in row:
            if spot.is_start() or spot.is_end() or spot.is_barrier():
                spot.draw(win)

    # draw_grid(win, rows, width)

    pygame.display.update()  # Aggiorna la finestra solo alla fine


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


def animate_airplane(start, plan, grid, rows):
    cell_size = (WIDTH) // rows  # Dimensione di una cella in pixel
    airplane_pixel_size = TRUCK_SIZE * cell_size  # Dimensione dell'aereo scalata
    scaled_airplane = pygame.transform.scale(
        TRUCK_ICON, (airplane_pixel_size, airplane_pixel_size))

    # Rimuove la traccia dei nodi aperti
    for row in grid:
        for spot in row:
            if spot.is_open() or spot.is_closed():  # Se era un nodo aperto
                spot.reset()  # Resettalo

    # Ottieni la posizione iniziale dell'aereo
    x, y = start.row, start.col

    # Resetta il punto di partenza originale
    grid[x][y].reset()

    # Animazione dell'aereo lungo il percorso
    for move, next_move in zip(plan, plan[1:]):
        pygame.time.delay(50)  # Ritardo per l'animazione

        # Resetta la cella precedente (solo se non è la base)
        if not grid[x][y].is_end():
            grid[x][y].reset()

        # Muove l'aereo nella direzione corretta
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

        # Imposta la nuova cella come start (aereo)
        grid[x][y].make_start()

        # Ridisegna la mappa con l'aereo nella nuova posizione
        WIN.blit(BACKGROUND_IMAGE, (0, 0))  # Ridisegna lo sfondo
        draw(WIN, grid, rows, WIDTH)  # Disegna gli altri elementi

        pygame.display.update()  # Aggiorna la finestra

    # L'animazione è terminata, lascia l'aereo fermo sopra la base
    print(f"L'aereo è arrivato alla base a ({x}, {y})")

    pygame.display.update()


def mark_expanded(exp, grid):
    for e in exp:
        grid[e[0]][e[1]].make_closed()


def save_to_file(grid, start, end, filename="temp.json"):
    barrier = list()
    for x in grid:
        for spot in x:
            if spot.is_barrier():
                barrier.append((spot.row, spot.col))
    res = {"rows": len(grid), "start": (start.row, start.col),
           "end": (end.row, end.col), "barrier": barrier}
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


# Variabile per tracciare l'euristica selezionata
selected_heuristic = "m"


def make_plan(p, draw, win, grid, rows, width, search_algorithm):
    if isinstance(search_algorithm, ASTARPathFinder) or isinstance(search_algorithm, IDASTARPathFinder):
        plan = search_algorithm.solve(p, lambda: draw(
            win, grid, rows, width), grid, selected_heuristic)
    else:
        plan = search_algorithm.solve(
            p, lambda: draw(win, grid, rows, width), grid)

    return plan


def update_selected_heuristic(event, search_algorithm):
    global selected_heuristic

    # Controlla se un bottone è stato cliccato e aggiorna l'euristica
    heuristic_selected = manhattan.click(event, search_algorithm, "m")
    if heuristic_selected:
        selected_heuristic = heuristic_selected
        # Deselect the other button
        chebyshev.change_text("chebyshev", bg="navy")
        euclidean.change_text("eucledian", bg="navy")
        blind.change_text("blind", bg="navy")

    heuristic_selected = chebyshev.click(event, search_algorithm, "c")
    if heuristic_selected:
        selected_heuristic = heuristic_selected
        # Deselect the other button
        manhattan.change_text("manhattan", bg="navy")
        euclidean.change_text("eucledian", bg="navy")
        blind.change_text("blind", bg="navy")

    heuristic_selected = euclidean.click(event, search_algorithm, "e")
    if heuristic_selected:
        selected_heuristic = heuristic_selected
        # Deselect the other button
        manhattan.change_text("manhattan", bg="navy")
        chebyshev.change_text("chebyshev", bg="navy")
        blind.change_text("blind", bg="navy")

    heuristic_selected = blind.click(event, search_algorithm, "b")
    if heuristic_selected:
        selected_heuristic = heuristic_selected
        # Deselect the other button
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
                    return heuristic_type  # Return the selected heuristic
        return None


save_map_button = Button(
    "Save Map",
    (WIDTH-200, 100),
    font=20,
    bg="navy",
    feedback="Saved")

manhattan = Button(
    "manhattan",
    (WIDTH-200, 200),
    font=20,
    bg="navy",
    feedback="manhattan <=")

chebyshev = Button(
    "chebyshev",
    (WIDTH-200, 250),
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
    (WIDTH-200, 350),
    font=20,
    bg="navy",
    feedback="blind <=")

clock = pygame.time.Clock()


@click.command()
@click.option('-w', '--width', default=WIDTH, help="Width of the Windows")
@click.option('-r', '--rows', default=50, help="Number of rows/columns in the map")
@click.option('-s', '--search_algorithm', default="ASTAR", help="Search algorithm to be used")
@click.option('-f', '--filename', default=None, help="Initialize map with data from file")
def main(width, rows, search_algorithm, filename=None):
    win = WIN
    start = None
    end = None
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
        grid, start, end, rows, wall = make_grid_from_file(filename, width)
        for i in list(wall):
            grid[i[0]][i[1]].make_barrier()
    else:
        grid = make_grid(rows, width)
        wall = set()
    run = True

    while run:
        draw(win, grid, rows, width)

        for event in pygame.event.get():
            pygame.display.update()
            if event.type == pygame.QUIT:
                run = False
            save_map_button.click_save(event, grid, start, end)
            update_selected_heuristic(event, search_algorithm)

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
                        spot.make_barrier()
                        wall.add((row, col))

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
                    elif spot.is_barrier() or spot.is_area():
                        wall.remove((row, col))
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_SPACE and start and end:
                    world = World(rows-1, rows-1, wall)
                    p = PathFinding((start.row, start.col),
                                    (end.row, end.col), world)
                    now = time.time()
                    plan = make_plan(p, draw, win, grid, rows,
                                     width, search_algorithm)
                    now = time.time() - now
                    print("Number of Expansion: {} in {} seconds".format(
                        search_algorithm.expanded, now))
                    if plan is not None:
                        print(plan)
                        print("Cost of the plan is: {}".format(len(plan)))
                        mark_spots(start, grid, plan)
                        animate_airplane(start, plan, grid, rows)
                    draw(win, grid, rows, width)
                if event.key == pygame.K_c:
                    start = None
                    end = None
                    grid = make_grid(rows, width)
                    wall = set()

    pygame.quit()


if __name__ == '__main__':
    main()
