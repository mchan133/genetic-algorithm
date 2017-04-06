#!/usr/bin/env python

# heuristic_search/grid.py
# Authors: Matthew Chan and Jeremy Savarin
# CS440: Project 1 - Heuristic Search

import pygame
import heuristic_search.astar as astar

from heuristic_search.gridfunctions import Grid

# Some global constants
# Set up colors for the grid
START = (255, 0, 0)
PATH = (0, 255, 0)
GOAL = (0, 200, 0)
BLOCKED = (0, 0, 0)
HARD_TO_TRAVERSE = (127, 127, 127)
UNBLOCKED = (255, 255, 255)
HIGHWAY_REGULAR = (24, 19, 178)
HIGHWAY_HTT = (114, 111, 198)

# square size for grid display
SQ_SZ = 7

'''
fn create_grid - draws a 120x160 grid map of squares representing different
types of terrain: unblocked, partially-blocked and blocked cells. There is also
a representation of grids where motion can be "accelerated" (like a river or a
highway).
'''


def create_grid(grid_obj, path=None, astar=None, proc=0):
    ROWS = grid_obj.ROWS
    COLS = grid_obj.COLS
    GRID_WIDTH = COLS * SQ_SZ
    GRID_HEIGHT = ROWS * SQ_SZ

    start = grid_obj.cells[0]
    goal = grid_obj.cells[1]
    lines = grid_obj.grid

    # Intitialize pygame
    pygame.init()

    # Create grid
    grid = pygame.display.set_mode((GRID_WIDTH, GRID_HEIGHT + 200))

    # Check if pygame initialized
    is_init = pygame.display.get_init()

    # Set window title
    pygame.display.set_caption('Heuristic search simulation')


    # Color cells by type
    color_cells(grid, lines, start, goal, (GRID_WIDTH, GRID_HEIGHT))

    c_tmp = None
    r_tmp = None
    vals = None

    toggle_path = False
    key_unpress = True

    clock = pygame.time.Clock()

    while True:
        # Poll for user events
        event = pygame.event.poll()
        clicked = pygame.mouse.get_pressed()
        new_pos = pygame.mouse.get_pos()

        if event.type == pygame.QUIT:
            break

        # Normalize for square size
        c = new_pos[0] // SQ_SZ
        r = new_pos[1] // SQ_SZ

        coord = (r, c)

        # Toggle A to show A* path
        key = pygame.key.get_pressed()
        
        if key[pygame.K_ESCAPE]:
            break

        if key[pygame.K_a] and key_unpress:
            key_unpress = False
            print("a press")
            if not toggle_path and astar:
                draw_path(grid, path, astar[3][proc])
                toggle_path = True
            else:
                color_cells(grid, lines, start, goal, (GRID_WIDTH, GRID_HEIGHT))
                draw_path(grid, path)
                toggle_path = False
        elif not key[pygame.K_a]: #checks let go of a
            if not key_unpress:
                print("a unpress")
                #proc = (proc + 1) % len(astar[3])
                #print(proc)
            key_unpress = True

        # mouse pos update
        if c_tmp != c or r_tmp != r:
            c_tmp = c
            r_tmp = r

            grid.fill(BLOCKED, (0, GRID_HEIGHT, GRID_WIDTH, 200))
            font = pygame.font.SysFont('verdana', 20, 16)

            if 0 <= r < ROWS and 0 <= c < COLS and astar:
                h = str(astar[0][proc][r][c])[:7]
                g = str(astar[1][proc][r][c])[:7]
                f = str(astar[2][proc][r][c])[:7]
                vals = "{0} | h:{1}, g:{2}, f:{3}".format(str(coord), h, g, f)
            else: vals="Nothing Found."

            fn_info = font.render(vals, False, UNBLOCKED)
            grid.blit(fn_info, (0, GRID_HEIGHT))

        pygame.display.flip()
        clock.tick(5)

    pygame.quit()

    return is_init



'''
fn color_cells - colors in cells on the grid based on terrain type
'''


def color_cells(surf, lines, start, goal, dims):
    # dims = (WIDTH, HEIGHT)
    # for refreshing the grid
    pygame.draw.rect(surf, BLOCKED, (0, 0, dims[0], dims[1]))

    # Color cells by type
    for i in range(0, len(lines)):
        for j in range(0, len(lines[i])):
            cell_coord = (j*SQ_SZ, i*SQ_SZ, SQ_SZ, SQ_SZ)

            if lines[i][j] == 'S':
                surf.fill(START, cell_coord)
            elif lines[i][j] == 'G':
                surf.fill(GOAL, cell_coord)
            elif lines[i][j] == 'O':
                surf.fill(BLOCKED, cell_coord)
            elif lines[i][j] == '1':
                surf.fill(UNBLOCKED, cell_coord)
            elif lines[i][j] == '2':
                surf.fill(HARD_TO_TRAVERSE, cell_coord)
            elif lines[i][j] == 'a':
                surf.fill(HIGHWAY_REGULAR, cell_coord)
            elif lines[i][j] == 'b':
                surf.fill(HIGHWAY_HTT, cell_coord)

    # Fill in start and goal cells
    start_coord = (start[1]*SQ_SZ, start[0]*SQ_SZ, SQ_SZ, SQ_SZ)
    surf.fill(START, start_coord)
    goal_coord = (goal[1]*SQ_SZ, goal[0]*SQ_SZ, SQ_SZ, SQ_SZ)
    surf.fill(GOAL, goal_coord)

'''
fn draw_line() - draws a line segment b/w neighborng cells
'''


def draw_line(surf, begin, end, color=START, lw=2):
    # offset, center of square
    ctr = SQ_SZ / 2

    # pygame draws in (x, y), args are in (r, c)
    b = (begin[1]*SQ_SZ + ctr, begin[0]*SQ_SZ + ctr)
    e = (end[1]*SQ_SZ + ctr, end[0]*SQ_SZ + ctr)

    pygame.draw.line(surf, color, b, e, lw)

'''
fn draw_path - draws path from starting cell to target cell
'''


def draw_path(surf, path, parents=None):
    if not path:
        # no path exists
        # do something
        print("no path exists")
        return 0

    if parents:
        for r in range(len(parents)):
            for c in range(len(parents[0])):
                if parents[r][c]:
                    draw_line(surf, (r, c), parents[r][c])

    for i in range(1, len(path)):
        draw_line(surf, path[i-1], path[i], PATH, 3)


def run_search(mode=True, filename=None, rand_state=None, heuristic=0, weight=1, s_type=0, ls=[1,2,3], w2=1):
    print("Setting up...")

    h_dict = {
            0:astar.h_function0,
            1:astar.h_function1,
            2:astar.h_function2,
            3:astar.h_function3,
            4:astar.h_function4,
            5:astar.h_function5}

    vals = None

    if mode:
        g = Grid(120, 160, rand_state=rand_state)
        # gets the relevant parameters from the grid
        vals = [g.cells[0], g.cells[1], g.grid]
    else:
        #vals = read_grid_from_file(filename)
        g = Grid.read_grid_from_file(filename)
        vals = [g.cells[0], g.cells[1], g.grid]

    heurs = []
    if s_type > 0:
        for i in ls:
            heurs.append(h_dict[i])


    print("Running A*...")
    results = -1
    if s_type == 0:
        results = astar.astar_search(g, h_dict[heuristic], weight)
    elif s_type == 1:
        results = astar.seq_astar_search(g, heurs, weight, w2)
    elif s_type == 2:
        results = astar.integ_astar_search(g, heurs, weight, w2)
    if results and results != -1:
        astar_path, a_params, p = results
        create_grid(g, path=astar_path, astar=a_params, proc=p)
    elif results == None:
        create_grid(g, path=None, astar=None)
    else:
        print("invalid input")


if __name__ == '__main__':
    run_search()
