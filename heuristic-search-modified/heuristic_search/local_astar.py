#!/usr/bin/env python

# heuristic_search/astar.py
# CS440: Project 1 - Heuristic Search
# Authors: Matthew Chan and Jeremy Savarin

import random
#import queue  # contains priority queue for astar
import sys
import time
import math
from my_pq import My_PQ


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
            return (len(visited), 0)

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

def local_tsp_init(cities):
    num_cities = len(cities)
    tour = []
    l = [i for i in range(num_cities)]
    random.shuffle(l)
    for i in range(num_cities-1):
        tour.append((l[i], l[i+1]))
    tour.append((l[-1], l[0]))

    return tour
    #num_cities = len(cities)
    #tour = [i for i in range(num_cities)]
    #random.shuffle(tour)

def local_tsp(tour, adj, tb=False, t=600, early=False):
    annealing = .50
    decrease  = .01
    dec_iter   = 50
    dist = 0
    new_dist = 0
    min_dist = float('inf')
    iters = 0
    num_c = 4 * len(tour) ** 2

    start_t = time.clock()

    counter = 0
    while (time.clock() - start_t) < t:

        success, new_dist = eval_swap(tour, adj, annealing)

        if counter % dec_iter == (dec_iter-1):
            annealing -= decrease
            if annealing < 0:
                annealing = 0

        if new_dist < dist:
            min_dist = new_dist
            iters = counter

        if early and annealing==0:
            if (counter-iters>num_c) and (dist > min_dist or math.isclose(dist, min_dist)):
                return tour, dist, (iters, min_dist, counter)

        dist = new_dist
        #print(dist)


        counter += 1

    #print(iters, min_dist)
    return tour, dist, (iters, min_dist, counter)



def swap_edges(tour, show_only=False):
    num_cities = len(tour)
    a=0
    b=0
    while True:
        a = random.randrange(num_cities)
        b = random.randrange(num_cities)
        if 2 < abs(a - b) < (num_cities-1):
            break;

    if show_only:
        return (a, b)

    e1 = tour[a]
    e2 = tour[b]
    n1 = (e1[0], e2[0])
    n2 = (e1[1], e2[1])

    tour.remove(e1)
    tour.remove(e2)
    tour.append(n1)
    tour.append(n2)

def eval_tour(tour, adj):
    dist = 0
    for e in tour:
        dist += adj[e[0]][e[1]]

    return dist

def eval_swap(tour, adj, anneal):
    dist = eval_tour(tour, adj)
    a, b = swap_edges(tour, show_only=True)
    e1 = tour[a]
    e2 = tour[b]
    n1 = (e1[0], e2[0])
    n2 = (e1[1], e2[1])
    new_dist = dist - adj[e1[0]][e1[1]] - adj[e2[0]][e2[1]] \
            + adj[n1[0]][n1[1]] + adj[n2[0]][n2[1]]
    
    if new_dist < ((1+anneal)*dist):
        tour.remove(e1)
        tour.remove(e2)
        tour.append(n1)
        tour.append(n2)
        return (True, new_dist)
    
    return (False, dist)

def eval_set_tour(tour, adj):
    dist = 0
    for i in range(len(tour)-2):
        dist += adj[i][i+1]

    return dist

def local_testbench(number, t=60, e=False):
    for i in range(25):
        cities = generate_tsp(number, rand_state=i)
        adj = init_adj(cities)
        for j in range(3):
            tour = local_tsp_init(cities)
            start_t = time.clock()
            tour, dist, xtra = local_tsp(tour, adj, t=t, early=e)
            stop_t = time.clock()
            print("%d\t%d\t%d\t%f\t%f\t%d\t%f" % (i, j, number, dist, xtra[1], xtra[2], stop_t-start_t))

def local_testbench_var(number, t=60, e=False): #variation
    cities = generate_tsp(number, rand_state=9001)
    for i in range(25):
        adj = init_adj(cities)
        tour = local_tsp_init(cities)
        start_t = time.clock()
        tour, dist, xtra = local_tsp(tour, adj, t=t, early=e)
        stop_t = time.clock()
        print("%d\t%f\t%f\t%d\t%f" % (number, dist, xtra[1], xtra[2], stop_t-start_t))



if __name__ == "__main__":

    #testbench(10)
    #local_testbench_var(25, t=10, e=True)
    local_testbench(int(sys.argv[1]), t=10, e=True)

    #cities = generate_tsp(10, rand_state=0)
    #print(cities)
    #adj = init_adj(cities)
    #start_t = time.clock()
    #result = astar_tsp(cities, adj, tb=False)
    #print("astar:", result, eval_set_tour(result, adj))

    #cities = generate_tsp(50, rand_state=1)
    #print(cities)
    #adj = init_adj(cities)
    #tour = local_tsp_init(cities)
    ##print(tour, eval_tour(tour, adj))
    ##swap_edges(tour)
    ##print(tour, eval_tour(tour, adj))
    #t, dist, xtra = local_tsp(tour, adj)
    #print("local:", tour, dist, xtra)

    #stop_t = time.clock()


