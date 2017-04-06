#!/usr/bin/env python

# heuristic_search/astar.py
# CS440: Project 1 - Heuristic Search
# Authors: Matthew Chan and Jeremy Savarin

import random
#import queue  # contains priority queue for astar
import sys
import time
# temporary workaround for 'module not found'
if sys.argv[0]=="app.py" or sys.argv[0]=="../app.py":
    from heuristic_search.gridfunctions import Grid
    from heuristic_search.my_pq import My_PQ
else:
    from gridfunctions import Grid
    from my_pq import My_PQ


def calc_cost(terrain, current, neighbor):
    v1 = current
    v2 = neighbor
    t1 = terrain[current[0]][current[1]]
    t2 = terrain[neighbor[0]][neighbor[1]]
    diagonal = ((v2[0]-v1[0]) != 0) and ((v2[1]-v1[1]) != 0)
    return cost_of([t1, t2], diagonal)


# helper for cost
def cost_of(charlist, diagonal):
    if len(charlist) > 2:
        return None

    cost = 0

    for char in charlist:
        if char == '0':
            cost += float('inf')
        if char == '1':
            cost += 2**.5 if diagonal else 1.0
        if char == '2':
            cost += 8**.5 if diagonal else 2.0
        if char == 'a':
            cost += .125**.5 if diagonal else .25
        if char == 'b':
            cost += .5**.5 if diagonal else .5

    if cost != 0:
        return cost / 2.0
    else:
        return None


# path length from start
# inefficient, no longer in use
'''
def cost_function(terrain, parent_grid, neighbor):
    current = neighbor
    parent = parent_grid[current[0]][current[1]]
    cost = 0

    while parent != current:
        v1 = current
        v2 = parent
        t1 = terrain[current[0]][current[1]]
        t2 = terrain[parent[0]][parent[1]]
        diagonal = ((v2[0]-v1[0]) != 0) and ((v2[1]-v1[1]) != 0)
        cost += cost_of([t1, t2], diagonal)

        current = parent
        parent = parent_grid[current[0]][current[1]]

    return cost
'''


# returns if a cell is in bounds
def in_bounds(point, r, c):
    if 0 <= point[0] < r:
        if 0 <= point[1] < c:
            return True

    return False


# traverses path back to start from goal (at end of astar)
def traverse_path(parent_grid, goal):
    current = goal
    parent = parent_grid[current[0]][current[1]]
    pathlist = []

    while parent != current:
        pathlist.append(current)
        current = parent
        parent = parent_grid[current[0]][current[1]]

    pathlist.append(current)  # starting node
    pathlist.reverse()
    return pathlist

def integ_astar_search(grid, heuristics=[], weight=1, w2=1, test_stats=False):
    # initializing variables
    terrain = grid.grid
    cols = grid.COLS
    rows = grid.ROWS
    start = grid.cells[0]
    goal = grid.cells[1]
    s = start  # placeholder variable
    g = goal

    if len(heuristics) < 2:
        print("not enough heuristics")
        return
    num_heurs = len(heuristics)
    inf = float('inf')

    # for test_stats
    fringe_size = []

    p_grids = []
    g_grids = []
    h_grids = []
    f_grids = []
    expanded = []
    fringe = []

    # one grid shared between searches (wrapped for compatibility)
    p_grids.append([[None for c in range(cols)] for r in range(rows)])
    g_grids.append([[inf  for c in range(cols)] for r in range(rows)])
    p_grids[0][s[0]][s[1]] = s
    g_grids[0][s[0]][s[1]] = 0

    # only 2 of these (admissible/inadmissable)
    for i in range(2):
        expanded.append(set())

    # initializing variables for all heuristics
    for i in range(num_heurs):
        h_grids.append([[inf for c in range(cols)] for r in range(rows)])
        f_grids.append([[inf for c in range(cols)] for r in range(rows)])
        compute_heuristic(h_grids[i], heuristics[i], goal, weight)

        fringe.append(My_PQ())

        f_grids[i][s[0]][s[1]] = h_grids[i][s[0]][s[1]]

        # fringe stored as (f, (r,c))
        fringe[i].put( (f_grids[i][s[0]][s[1]], s) )

        # testing
        fringe_size.append(0)

    start_t = time.clock()

    while not fringe[0].empty():
        minkey_0 = fringe[0].peek()  # gets from fringe (not pop)
        while minkey_0[1] in expanded[0]:
            print("already expanded", minkey_0)
            fringe[0].get()
            minkey_0 = fringe[0].peek()

        if test_stats:
            fringe_size[0] = max(fringe_size[0], fringe[0].qsize())

        for i in range(1,num_heurs):
            if test_stats:
                fringe_size[i] = max(fringe_size[i], fringe[i].qsize())

            minkey_i = fringe[i].peek()
            while minkey_i and minkey_i[1] in expanded[1]:
                print("already expanded", minkey_i)
                fringe[i].get()
                minkey_i = fringe[i].peek()

            if minkey_i and minkey_i[0] < w2*minkey_0[0]:
                if g_grids[0][g[0]][g[1]] < minkey_i[0]:
                    if g_grids[0][g[0]][g[1]] < inf:
                        # found a path in search i
                        found_path = traverse_path(p_grids[0], goal)
                        grids = (h_grids, g_grids, f_grids, p_grids)

                        if test_stats:
                            search_t = time.clock() - start_t
                            stats = (fringe_size, expanded, search_t, \
                                    g_grids[0][g[0]][g[1]])
                            return (found_path, grids, stats, 0)
                        else:
                            return (found_path, grids, 0)
                else:  # expand current node
                    #print("expanding", i, minkey_0, minkey_i)
                    cur = minkey_i[1] #current cell
                    expanded[1].add(cur)

                    # removing from all fringes
                    for j in range(num_heurs):
                        fringe[j].remove(cur)

                    for r in range(-1, 2):
                        for c in range(-1, 2):
                            # neighboring cell
                            n = (cur[0]+r, cur[1]+c)

                            if not in_bounds(n, rows, cols):
                                continue
                            if n in expanded[1]:  # also catches self
                                continue
                            if terrain[n[0]][n[1]] == '0':  # blocked
                                continue

                            temp_cost = g_grids[0][cur[0]][cur[1]] + \
                                    calc_cost(terrain, cur, n)

                            #if fringe[i].contains(n):  # a path already known
                            if g_grids[0][n[0]][n[1]] < float('inf'):
                                if temp_cost >= g_grids[0][n[0]][n[1]]:
                                    continue

                            p_grids[0][n[0]][n[1]] = cur
                            g_grids[0][n[0]][n[1]] = temp_cost

                            if n not in expanded[0]:
                                key0 = f_fcn(g_grids[0], h_grids[0], n)
                                f_grids[0][n[0]][n[1]] = key0
                                fringe[0].put((key0, n))
                                
                                if n not in expanded[1]:
                                    for j in range(1, num_heurs):
                                        keyi = f_fcn(g_grids[0], h_grids[j], n)
                                        if keyi < w2*key0:
                                            f_grids[j][n[0]][n[1]] = keyi
                                            fringe[j].put((keyi, n))

            else:  # expanding state in admissible heuristic
                if g_grids[0][g[0]][g[1]] < minkey_0[0]:
                    if g_grids[0][g[0]][g[1]] < inf:
                        # found a path in search 0
                        found_path = traverse_path(p_grids[0], goal)
                        grids = (h_grids, g_grids, f_grids, p_grids)

                        if test_stats:
                            search_t = time.clock() - start_t
                            stats = (fringe_size, expanded, \
                                    search_t, g_grids[0][g[0]][g[1]])
                            return (found_path, grids, stats, 0)
                        else:
                            return (found_path, grids, 0)
                else:  # expand current node
                    #print("expanding", i, minkey_0, minkey_i)
                    cur = fringe[0].get()[1] #current cell
                    expanded[0].add(cur)

                    # removing from all fringes
                    for j in range(num_heurs):
                        fringe[j].remove(cur)

                    for r in range(-1, 2):
                        for c in range(-1, 2):
                            # neighboring cell
                            n = (cur[0]+r, cur[1]+c)

                            if not in_bounds(n, rows, cols):
                                continue
                            if n in expanded[0]:  # also catches self
                                continue
                            if terrain[n[0]][n[1]] == '0':  # blocked
                                continue

                            temp_cost = g_grids[0][cur[0]][cur[1]] + \
                                    calc_cost(terrain, cur, n)

                            #if fringe[0].contains(n):  # a path already known
                            if g_grids[0][n[0]][n[1]] < float('inf'):
                                if temp_cost >= g_grids[0][n[0]][n[1]]:
                                    continue

                            p_grids[0][n[0]][n[1]] = cur
                            g_grids[0][n[0]][n[1]] = temp_cost

                            if n not in expanded[0]:
                                key0 = f_fcn(g_grids[0], h_grids[0], n)
                                f_grids[0][n[0]][n[1]] = key0
                                fringe[0].put((key0, n))
                                
                                if n not in expanded[1]:
                                    for j in range(1, num_heurs):
                                        keyi = f_fcn(g_grids[0], h_grids[j], n)
                                        if keyi < w2*key0:
                                            f_grids[j][n[0]][n[1]] = keyi
                                            fringe[j].put((keyi, n))

    print("reached here")
    return None


def seq_astar_search(grid, heuristics=[], weight=1, w2=1, test_stats=False):
    # initializing variables
    terrain = grid.grid
    cols = grid.COLS
    rows = grid.ROWS
    start = grid.cells[0]
    goal = grid.cells[1]
    s = start  # placeholder variable
    g = goal

    if len(heuristics) < 2:
        print("not enough heuristics")
        return
    num_heurs = len(heuristics)
    inf = float('inf')

    # for test_stats
    fringe_size = []

    p_grids = []
    g_grids = []
    h_grids = []
    f_grids = []
    expanded = []
    fringe = []

    # initializing variables for all heuristics
    for i in range(num_heurs):
        p_grids.append([[None for c in range(cols)] for r in range(rows)])
        g_grids.append([[inf  for c in range(cols)] for r in range(rows)])
        h_grids.append([[inf  for c in range(cols)] for r in range(rows)])
        f_grids.append([[inf  for c in range(cols)] for r in range(rows)])
        compute_heuristic(h_grids[i], heuristics[i], goal, weight)

        expanded.append(set())
        fringe.append(My_PQ())

        p_grids[i][s[0]][s[1]] = s
        g_grids[i][s[0]][s[1]] = 0
        f_grids[i][s[0]][s[1]] = h_grids[i][s[0]][s[1]]

        # fringe stored as (f, (r,c))
        fringe[i].put( (f_grids[i][s[0]][s[1]], s) )

        # testing
        fringe_size.append(0)

    start_t = time.clock()

    while not fringe[0].empty():
        minkey_0 = fringe[0].peek()  # gets from fringe (not pop)
        while minkey_0[1] in expanded[0]:
            #print("already expanded", minkey_0)
            fringe[0].get()
            minkey_0 = fringe[0].peek()

        if test_stats:
            fringe_size[0] = max(fringe_size[0], fringe[0].qsize())

        for i in range(1,num_heurs):
            if test_stats:
                fringe_size[i] = max(fringe_size[i], fringe[i].qsize())

            minkey_i = fringe[i].peek()
            while minkey_i and minkey_i[1] in expanded[i]:
                #print("already expanded", minkey_i)
                fringe[i].get()
                minkey_i = fringe[i].peek()

            if minkey_i and minkey_i[0] < w2*minkey_0[0]:
                if g_grids[i][g[0]][g[1]] < minkey_i[0]:
                    if g_grids[i][g[0]][g[1]] < inf:
                        # found a path in search i
                        found_path = traverse_path(p_grids[i], goal)
                        grids = (h_grids, g_grids, f_grids, p_grids)

                        if test_stats:
                            search_t = time.clock() - start_t
                            stats = (fringe_size, expanded, search_t, \
                                    g_grids[i][g[0]][g[1]])
                            return (found_path, grids, stats, i)
                        else:
                            return (found_path, grids, i)
                else:  # expand current node
                    #print("expanding",i, minkey_0, minkey_i)
                    cur = minkey_i[1] #current cell
                    expanded[i].add(cur)

                    for r in range(-1, 2):
                        for c in range(-1, 2):
                            # neighboring cell
                            n = (cur[0]+r, cur[1]+c)

                            if not in_bounds(n, rows, cols):
                                continue
                            if n in expanded[i]:  # also catches self
                                continue
                            if terrain[n[0]][n[1]] == '0':  # blocked
                                continue

                            temp_cost = g_grids[i][cur[0]][cur[1]] + \
                                    calc_cost(terrain, cur, n)

                            if fringe[i].contains(n):  # a path already known
                                if temp_cost >= g_grids[i][n[0]][n[1]]:
                                    continue

                            p_grids[i][n[0]][n[1]] = cur
                            g_grids[i][n[0]][n[1]] = temp_cost
                            f_grids[i][n[0]][n[1]] = f_fcn(g_grids[i], h_grids[i], n)

                            fringe[i].put((f_grids[i][n[0]][n[1]], n))

            else:  # expanding state in admissible heuristic
                if g_grids[0][g[0]][g[1]] < minkey_0[0]:
                    if g_grids[0][g[0]][g[1]] < inf:
                        # found a path in search 0
                        found_path = traverse_path(p_grids[0], goal)
                        grids = (h_grids, g_grids, f_grids, p_grids)

                        if test_stats:
                            search_t = time.clock() - start_t
                            stats = (fringe_size, expanded, \
                                    search_t, g_grids[0][g[0]][g[1]])
                            return (found_path, grids, stats, 0)
                        else:
                            return (found_path, grids, 0)
                else:  # expand current node
                    #print("expanding", i, minkey_0, minkey_i)
                    cur = fringe[0].get()[1] #current cell
                    expanded[0].add(cur)

                    for r in range(-1, 2):
                        for c in range(-1, 2):
                            # neighboring cell
                            n = (cur[0]+r, cur[1]+c)

                            if not in_bounds(n, rows, cols):
                                continue
                            if n in expanded[0]:  # also catches self
                                continue
                            if terrain[n[0]][n[1]] == '0':  # blocked
                                continue

                            temp_cost = g_grids[0][cur[0]][cur[1]] + \
                                    calc_cost(terrain, cur, n)

                            if fringe[0].contains(n):  # a path already known
                                if temp_cost >= g_grids[0][n[0]][n[1]]:
                                    continue

                            p_grids[0][n[0]][n[1]] = cur
                            g_grids[0][n[0]][n[1]] = temp_cost
                            f_grids[0][n[0]][n[1]] = f_fcn(g_grids[0], h_grids[0], n)

                            fringe[0].put((f_grids[0][n[0]][n[1]], n))

    print("reached here")
    return None

def astar_search(grid, heuristic=None, weight=1, test_stats=False):
    # setup
    terrain = grid.grid
    cols = grid.COLS
    rows = grid.ROWS
    start = grid.cells[0]
    goal = grid.cells[1]
    s = start  # placeholder variables
    g = goal
    if not heuristic:
        heuristic = h_function0
    inf = float('inf')

    # for test_stats
    fringe_size = 0

    # initializing variables for a*
    p_grid = grid.parents
    g_grid = grid.values[0]
    h_grid = grid.values[1]
    f_grid = grid.values[2]

    compute_heuristic(h_grid, heuristic, g, weight)

    expanded = set()
    fringe = My_PQ()

    p_grid[s[0]][s[1]] = s
    g_grid[s[0]][s[1]] = 0
    f_grid[s[0]][s[1]] = h_grid[s[0]][s[1]]

    fringe.put((f_grid[s[0]][s[1]], s))

    start_t = time.clock()

    while not fringe.empty():
        # for testing
        if test_stats:
            fringe_size = max(fringe_size, fringe.qsize())
        # get from fringe and expand
        current = fringe.get()[1]  # strip off the f-value
        if current in expanded:
            continue
        expanded.add(current)

        # check for the goal (g)
        if current == goal:
            # wrap vals w/ list for compatibility with seq.
            found_path = traverse_path(p_grid, g)
            grids = ([h_grid], [g_grid], [f_grid], [p_grid])

            if test_stats:
                search_t = time.clock() - start_t
                stats = ([fringe_size], [len(expanded)], \
                        search_t, g_grid[g[0]][g[1]])
                return (found_path, grids, stats, 0)
            else:
                return (found_path, grids, 0)

        # get neighbors, add to fringe (calc f value)
        for i in range(-1, 2):
            for j in range(-1, 2):
                neighbor = (current[0]+i, current[1]+j)

                if not in_bounds(neighbor, rows, cols):
                    continue
                if neighbor in expanded:  # also catches self
                    continue
                if terrain[neighbor[0]][neighbor[1]] == '0':  # blocked
                    continue

                temp_cost = g_grid[current[0]][current[1]] + calc_cost(terrain, current, neighbor)

                #if neighbor in fringe_check:  # a path already known
                if fringe.contains(neighbor):
                    if temp_cost >= g_grid[neighbor[0]][neighbor[1]]:
                        continue

                p_grid[neighbor[0]][neighbor[1]] = current
                g_grid[neighbor[0]][neighbor[1]] = temp_cost
                f_grid[neighbor[0]][neighbor[1]] = f_fcn(g_grid, h_grid, neighbor)

                fringe.put((f_grid[neighbor[0]][neighbor[1]], neighbor))
                #fringe_check.add(neighbor)

    return None  #No path

def f_fcn(g, h, cell):
    return g[cell[0]][cell[1]] + h[cell[0]][cell[1]]

def compute_heuristic(h_grid, heuristic, goal, weight=1):
    for r in range(len(h_grid)):  # initializing h_grid w/ heuristic
        for c in range(len(h_grid[0])):
            h_grid[r][c] = weight * heuristic((r, c), goal)

# Heuristics
def h_function0(cell, goal):
    return 0  # uniform cost search (uninformed)


########

def invert(current, num_cities):
    l = []
    for i in range(num_cities):
        if i not in current:
            l.append(i)

    return l



def astar_tsp(city_list, adj, tb=False):
    # setup
    inf = float('inf')
    num_cities = len(city_list)
    counter = 0


    visited = set()
    fringe = My_PQ()

    # (f-val, ((visited),g)
    fringe.put( (0, (tuple([0]), 0)) )

    start_t = time.clock()

    while not fringe.empty():
        # get from fringe and expand
        node = fringe.get()
        current = node[1][0]
        g_val = node[1][1]
        #print(current)

        if time.clock() - start_t > 600: # 10 min
            return 0

        # not consistent, so may go over
        if len(current) > num_cities+1:
            continue
        if current in visited:
            continue
        visited.add(current)

        # check for the goal (g)
        if len(current) == num_cities+1:
            if(current[-1] == 0):
                for i in range(len(current)):
                    if i not in current:
                        continue
                if tb: return (len(visited), 1)
                return current


        # get neighbors, add to fringe (calc f value)
        for i in range(num_cities):
            if len(current) == num_cities:
                if i > 0:
                    continue
            elif i in current:
                continue

            neighbor = current + tuple([i])
            unvisited = invert(current, num_cities) #faster?

            h_val = prim(unvisited, adj) + find_closest(0, unvisited, adj)
            #h_val = find_closest(i, unvisited, adj)
            new_g = g_val + adj[current[-1]][i]
            f_val = new_g + h_val

            fringe.put((f_val, (neighbor, new_g)))

    return (len(visited), 0)  #No path

def generate_tsp(cities, rand_state=None):

    if rand_state != None:
        random.seed(rand_state)

    l = []  # list
    c_list = set()
    rand = random.randrange  # localizing function
    for i in range(cities):
        while True:
            x = rand(100)
            y = rand(100)
            if (x,y) not in c_list:
                break
        l.append((i, (x, y)))
        c_list.add((x,y))

    return l

def init_adj(cities):  # create adjacency matrix
    num = len(cities)
    inf = float('inf')
    res = [[0 for i in range(num)] for j in range(num)]
    
    for i in range(num):
        for j in range(num):
            res[i][j] = calc_dist(cities[i][1], cities[j][1])
            res[j][i] = res[i][j]

    for i in range(num):
        res[i][i] = inf

    return res

def calc_dist(a, b):
    return ((a[0]-b[0])**2 + (a[1]-b[1])**2)**.5

def closest(a, adj):
    return min(adj[a][:])

def find_closest(a, others, adj):
    min_dist = 1000
    closest = -1
    for i in others:
        if adj[a][i] < min_dist:
            min_dist = adj[a][i]
            closest = i
    return closest

def prim(cities, adj, retmst=False): #cities is list of ints

    if len(cities) == 0:
        return 0

    mst_set = set()
    mst = []
    mst_dist = 0

    city = cities.pop()
    mst_set.add(city)
    while len(cities) > 0:

        min_edge = float('inf')
        addition = None
        for i in mst_set:
            for j in cities:
                if adj[i][j] < min_edge:
                    min_edge = adj[i][j]
                    addition = (i,j)


        mst_dist += min_edge
        mst.append(addition)
        mst_set.add(addition[1])
        cities.remove(addition[1])

    #print(mst)
    if retmst:
        return (mst, mst_dist)
    return mst_dist

def print_tsp_grid(cities):

    l = []
    for city in cities:
        l.append(city[1])


    dim = 100

    print('\t+' + '-'*dim + '+')

    y = 0
    while y < dim:
        print(str(y) + '\t|', end='')

        x = 0
        while x < dim:
            if (x,y) in l:
                city = str(l.index((x,y)))
                print(city, end='')
                x+=len(city)
            else:
                print('.', end='')
                x += 1

        print('|')
        y += 1

    print('\t+' + '-'*dim + '+')

def testbench(number):

    for i in range(25):
        cities = generate_tsp(number, rand_state=i)
        adj = init_adj(cities)
        start_t = time.clock()
        expanded, success = astar_tsp(cities, adj, tb=True)
        stop_t = time.clock()
        print("%d\t%d\t%d\t%f" % (number, success, expanded, stop_t-start_t))


if __name__ == "__main__":

    testbench(10)

    #cities = generate_tsp(25, rand_state=1)
    #adj = init_adj(cities)
    #l2 = [i for i in range(len(cities))]
    #mst, mst_dist = prim(l2, adj, True)
    #print(cities)
    #print(mst)
    #print_tsp_grid(cities)
    #start_t = time.clock()
    #result = astar_tsp(cities, adj)
    #stop_t = time.clock()
    #print(result, stop_t-start_t)

    #g = Grid(120, 160, rand_state=2)
    #    
    #import cProfile
    #import pstats

    #heurs = [h_function1, h_function2, h_function3, h_function4, h_function5]
    ##heurs = [h_function1]

    #pr = cProfile.Profile()
    #pr.enable()
    ##r = integ_astar_search(g, heurs, w2=1.5)
    #r = seq_astar_search(g, heurs, w2=1)
    ##r = astar_search(g)
    #pr.disable()
    #pstats.Stats(pr).print_stats()
    #print(r[0])
    #print(g.cells)


