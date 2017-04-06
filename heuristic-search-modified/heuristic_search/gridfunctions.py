#!/usr/bin/env python

# heuristic_search/gridfunctions.py
# CS440: Project 1 - Heuristic Search
# Authors: Matthew Chan and Jeremy Savarin

import random
import copy
import time

'''
class Grid - class to encompass data representation of grid world and build
grid world from scratch. The following represents the type of squares in
the grid:

    * 0 - blocked cell
    * 1 - unblocked cell
    * 2 - hard-to-traverse cell
    * a - highway cell (regular)
    * b - highway cell (hard-to-traverse)
'''


class Grid:

    def __init__(self, rows=120, cols=160, rand_state=None, add_blocked=True,
                 p_b=.2, add_htt=True, n_anchors=8, p_a=0.5, add_highways=True,
                 n_highways=4, h_length=100, h_segment=20):
        # Choose a seed value for repeatable results
        if rand_state != None: # without none, misses 0
            random.seed(rand_state)

        self.ROWS = rows
        self.COLS = cols

        # Initialize all cells as unblocked
        self.grid = [['1' for c in range(cols)] for r in range(rows)]

        self.htt_cells = self.add_hardtraverse(n_anchors, p_a) \
            if add_htt else []
        self.highwaylist = self.add_highways(n_highways, h_length, h_segment) \
            if add_highways else []
        if add_blocked:
            self.add_blocked_cells(p_b)
        self.cells = self.add_start_goal_cells()

        INF = float('inf')
        # grid containing traversal parents (r,c tuples)
        self.parents = [[None for c in range(cols)] for r in range(rows)]
        # grid containing g, h, and f values from a* traversal
        self.values = [[[INF for c in range(cols)] for r in range(rows)] for val in range(3)]

    '''
    fn generate_probability(p) - returns True with probability p, False
    with probability 1-p.
    '''
    def generate_probability(self, p):
        if (p < 0 or p > 1):
            raise ValueError("p can only be between 0 and 1.")

        return random.random() < p


    '''
    fn print_grid() - prints data representation of grid world to stdout
    '''
    def print_grid(self, pretty_print=False):
        vals = {
            '0': 'X',
            '1': '.',
            '2': '^',
            'a': 'H',
            'b': '#',
            'S': 'S',
            'G': 'G'
        }

        print('\t+' + '-' * self.COLS + '+')

        for r in range(self.ROWS):
            print(str(r) + '\t|', end='')

            for c in range(self.COLS):
                if pretty_print:
                    print(vals[self.grid[r][c]], end='')
                    if self.htt_cells and (r, c) in self.htt_cells:
                        print("\b*", end='')
                else:
                    print(self.grid[r][c], end='')

            print('|')

        print('\t+' + '-' * self.COLS + '+')
        if pretty_print:
            print('Size (r, c):', (self.ROWS, self.COLS))
            print('start, goal:', self.cells)

    '''
    fn add_hardtraverse(n_anchors=8, p=0.5) - adds hard-to-traverse cells to
    grid. Returns list of tuples representing the anchors (row,col).
    '''
    def add_hardtraverse(self, n_anchors=8, p=0.5):
        htt_cells = []

        i = 0
        while i < n_anchors:
            # Select random anchor points
            while(1):
                r = random.randrange(self.ROWS)
                c = random.randrange(self.COLS)
                if not (r, c) in htt_cells:
                    htt_cells.append((r, c))
                    i += 1
                    break

            # Consider 31x31 region centered around anchors, accounting for
            # hitting the boundaries prematurely
            rmin = max(0, r - 15)
            rmax = min(self.ROWS, r + 15)
            cmin = max(0, c - 15)
            cmax = min(self.COLS, c + 15)

            # Mark cells in region with 50% probability
            for r in range(rmin, rmax):
                for c in range(cmin, cmax):
                    if self.generate_probability(p):
                        self.grid[r][c] = '2'

        return htt_cells

    '''
    fn add_highways(n_highways=4) - adds highway cells to the grid to allow for
    "faster" movement. Returns a list of tuples containing highway vertex
    coordinates ((begin_row, begin_col),(end_row, end_col)).
    '''
    def add_highways(self, n_highways=4, h_length=100, h_segment=20):
        highwaylist = []
        attempts = 0
        MAX_ATTEMPTS = 10

        # Try to build highways
        while len(highwaylist) < n_highways:
            # Restart highway building process after MAX_ATTEMPTS
            if attempts > MAX_ATTEMPTS:
                highwaylist = []  # restarting

            new_highway = self.build_highway(h_length, h_segment)

            # Check for conflicts in built highway before appending
            if not self.has_conflicts(new_highway, highwaylist):
                highwaylist.append(new_highway)
                attempts = 0

            attempts += 1

        # Add highways to grid
        for highway in highwaylist:
            # Add individual segments
            for segment in highway:
                b = segment[0]  # begin
                e = segment[1]  # end

                if e[1] == b[1]:  # vertical
                    direction = (1 if (e[0]-b[0] > 0) else -1)
                    for disp in range(b[0], e[0] + direction, direction):
                        r = disp
                        c = b[1]

                        if self.is_htt_cell(r, c):
                            self.grid[r][c] = 'b'
                        else:
                            self.grid[r][c] = 'a'

                else:  # horizontal
                    direction = (1 if (e[1]-b[1] > 0) else -1)
                    for disp in range(b[1], e[1] + direction, direction):
                        r = b[0]
                        c = disp

                        if self.is_htt_cell(r, c):
                            self.grid[r][c] = 'b'
                        else:
                            self.grid[r][c] = 'a'

        return highwaylist


    def build_highway(self, h_length, h_segment):
        highway = []
        # a highway contains: ( (start-coords), (stop-coords) ) for each
        # segment of the highway

        startside = random.randrange(4)  # 0 is top, going clockwise

        if startside % 2 == 0:
            sidelength = self.COLS
        else:
            sidelength = self.ROWS

        cell_coord = random.randrange(1, sidelength-1)  # no corners

        row = -1
        col = -1
        direction = -1
        #   0
        # 3 + 1
        #   2

        if startside == 0:
            row = 0
            col = cell_coord
        elif startside == 1:
            row = cell_coord
            col = self.COLS - 1
        elif startside == 2:
            row = self.ROWS - 1
            col = cell_coord
        elif startside == 3:
            row = cell_coord
            col = 0
        direction = (startside + 2) % 4

        done = False
        start = (row, col)

        while not done:
            if direction == 0:
                stop = (start[0] - h_segment, start[1])
            if direction == 1:
                stop = (start[0], start[1] + h_segment)
            if direction == 2:
                stop = (start[0] + h_segment, start[1])
            if direction == 3:
                stop = (start[0], start[1] - h_segment)

            # checking in bounds
            if (0 < stop[1] < (self.COLS-1)) and (0 < stop[0] < (self.ROWS-1)):
                highway.append((start, stop))
                start = stop  # for the next segment
                if self.generate_probability(.4):  # change direction
                    if self.generate_probability(.5):
                        direction = (direction + 1) % 4
                    else:
                        direction = (direction - 1) % 4

            else:  # at/past an edge
                if stop[0] <= 0:
                    stop = (0, stop[1])
                elif stop[0] >= self.ROWS:
                    stop = (self.ROWS-1, stop[1])
                if stop[1] <= 0:
                    stop = (stop[0], 0)
                elif stop[1] >= self.COLS:
                    stop = (stop[0], self.COLS-1)
                highway.append((start, stop))
                # TODO: check conflicts  & length req
                if not self.has_conflicts(highway, h_length=h_length):
                    done = True

            # checking for self-intersections every iteration
            for i in range(len(highway) - 2):
                if self.check_conflict(highway[-1], highway[i]):
                    highway = []
                    direction = (startside + 2) % 4
                    start = (row, col)
                    done = False
                    break

        return highway

    def has_conflicts(self, highway, highwaylist=None, h_length=100):
        # selfcheck will also do length check
        selfcheck = False

        if highwaylist is None:
            highwaylist = [highway]
            selfcheck = True

        if selfcheck:  # length requirement only
            h_len = 0
            for segment in highway:
                h_len += abs(segment[1][0] - segment[0][0]) + \
                         abs(segment[1][1] - segment[0][1])
            if h_len < h_length:
                return True
            else:
                return False

        for i in range(len(highwaylist)):
            for s1 in range(len(highwaylist[i])):
                for s2 in range(len(highway)):
                    if self.check_conflict(highwaylist[i][s1], highway[s2]):
                        return True

        return False

    def check_conflict(self, segment1, segment2):
        s1 = segment1
        s2 = segment2
        c_cross = False
        r_cross = False
        diff1r = s1[1][0] - s1[0][0]  # vert
        diff1c = s1[1][1] - s1[0][1]  # horiz
        diff2r = s2[1][0] - s2[0][0]  # vert
        diff2c = s2[1][1] - s2[0][1]  # horiz

        # checking for row overlap
        # check if vertexes of s2 are in s1
        if min(s1[0][0], s1[1][0]) <= s2[0][0] <= max(s1[0][0], s1[1][0]):
            r_cross = True
        if min(s1[0][0], s1[1][0]) <= s2[1][0] <= max(s1[0][0], s1[1][0]):
            r_cross = True
        if min(s2[0][0], s2[1][0]) <= s1[0][0] <= max(s2[0][0], s2[1][0]):
            r_cross = True
        if min(s2[0][0], s2[1][0]) <= s1[1][0] <= max(s2[0][0], s2[1][0]):
            r_cross = True

        # checking for column overlap
        # check if vertexes of s2 are in s1
        if min(s1[0][1], s1[1][1]) <= s2[0][1] <= max(s1[0][1], s1[1][1]):
            c_cross = True
        if min(s1[0][1], s1[1][1]) <= s2[1][1] <= max(s1[0][1], s1[1][1]):
            c_cross = True
        if min(s2[0][1], s2[1][1]) <= s1[0][1] <= max(s2[0][1], s2[1][1]):
            c_cross = True
        if min(s2[0][1], s2[1][1]) <= s1[1][1] <= max(s2[0][1], s2[1][1]):
            c_cross = True

        return (r_cross and c_cross)

    '''
    fn is_highway_cell(row, col) - Returns True if grid[row][col] is a highway
    cell ,and False otherwise.
    '''
    def is_highway_cell(self, row, col):
        cell = self.grid[row][col]
        return cell == 'a' or cell == 'b'

    '''
    fn is_htt_cell(row, col) - Returns True if grid[row][col] is a
    hard-to-traverse cell, and False otherwise.
    '''
    def is_htt_cell(self, row, col):
        cell = self.grid[row][col]
        return cell == '2'

    '''
    fn is_blocked_cell(row, col) - Returns True if grid[row][col] is a
    blocked cell, and False otherwise.
    '''
    def is_blocked_cell(self, row, col):
        cell = self.grid[row][col]
        return cell == '0'

    def add_blocked_cells(self, p=.2):
        # 20% of cells are blocked
        BLOCKED_CELLS = p * self.ROWS * self.COLS

        i = 0

        while i < BLOCKED_CELLS:
            # Generate random cell
            r = random.randint(0, self.ROWS-1)
            c = random.randint(0, self.COLS-1)

            # Check if highway
            if not self.is_highway_cell(r, c):
                self.grid[r][c] = '0'
                i += 1

    '''
    fn add_start_goal_cells() - adds start and goal cells to the grid. Returns
    a list of tuples representing their coordinates in the grid.
    '''
    def add_start_goal_cells(self):
        # Try to place start cell
        is_start_placed = False
        on_sides = False

        cells = []

        while not is_start_placed:
            # Place in top left of grid
            # Guarantees correct spacing
            if self.generate_probability(.5):
                r = random.randint(0, 19)
                c = random.randint(0, self.COLS-1)
            else:
                r = random.randint(0, self.ROWS-1)
                c = random.randint(0, 19)
                on_sides = True

            if not self.is_blocked_cell(r, c):
                cells.append((r, c))
                is_start_placed = True

        # Try to place goal cell
        is_goal_placed = False

        while not is_goal_placed:
            # Place opposite of start
            if on_sides:
                r = random.randint(0, self.ROWS-1)
                c = random.randint(self.COLS-20, self.COLS-1)
            else:
                r = random.randint(self.ROWS-20, self.ROWS-1)
                c = random.randint(0, self.COLS-1)

            if not self.is_blocked_cell(r, c):
                cells.append((r, c))
                is_goal_placed = True

        return cells

    '''
    fn print_grid_to_file(filename) - prints grid data to file given by
    filename
    '''
    def print_grid_to_file(self, filename):
        with open(filename, 'w') as file:
            # Print start and goal coordinates
            file.write(str(self.cells[0]) + '\n')
            file.write(str(self.cells[1]) + '\n\n')

            # Print anchors in hard-to-traverse cells
            file.write('hard-to-traverse centers: ' + '\n')

            for point in self.htt_cells:
                file.write(str(point) + '\n')

            # Print data representation of the grid
            file.write('grid: ' + '\n')

            for r in range(self.ROWS):
                for c in range(self.COLS):
                    file.write(self.grid[r][c])

                file.write('\n')

    def read_grid_from_file(filename):
        lines = []

        with open(filename, 'r') as file:
            for line in file:
                lines.append(line.strip())

        # Read in start and goal
        st = lines[0].replace('(', '').replace(')', '').split(',')
        gl = lines[1].replace('(', '').replace(')', '').split(',')

        start = (int(st[0]), int(st[1]))
        goal = (int(gl[0]), int(gl[1]))
        cells = (start, goal)

        htt_str = "hard-to-traverse centers:"
        grid_str = "grid:"

        # taking htt centers
        htt_cells = []
        for i in range(lines.index(htt_str)+1, lines.index(grid_str)):
            line = lines[i]
            htt = line.replace('(', '').replace(')', '').split(',')
            htt_cells.append((int(htt[0]), int(htt[1])))

        # Only take lines relating to grid
        grid_lines = lines[(lines.index(grid_str)+1):]
        ROWS = len(grid_lines)
        COLS = len(grid_lines[0])
        terr = [[grid_lines[r][c] for c in range(COLS)] \
                for r in range(ROWS)]

        grid = Grid(ROWS,COLS,add_blocked=False, add_htt=False, add_highways=False)

        grid.grid = terr
        grid.cells = cells
        grid.htt_cells = htt_cells

        return grid

    def replace_start_and_goal(self):
        new_cells = self.add_start_goal_cells()
        self.cells = new_cells

        return new_cells


if __name__ == "__main__":
    g = Grid(120, 160, rand_state=0)
    g.print_grid(pretty_print=True)
    g.print_grid_to_file('test.txt')
    g2 = Grid.read_grid_from_file('test.txt')
    g2.print_grid(pretty_print=True)
    

    print(g.replace_start_and_goal())
    print(g.replace_start_and_goal())
    print(g.replace_start_and_goal())


    import sys  # testing size of node
    print("Size of tuple (r,c):", sys.getsizeof((999,999)))
    print("Size of fringe obj (priority,(r,c)):", sys.getsizeof((999,(999,999))))
    print("Size of int:", sys.getsizeof(1))



