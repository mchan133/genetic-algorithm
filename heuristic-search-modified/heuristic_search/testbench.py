from gridfunctions import Grid
import astar
import sys

'''
Usage: python testbench.py > textfile
Deterministic stats, to be processed in Excel
'''

#print("Size of tuple (r,c):", sys.getsizeof((999,999)))
#print("Size of fringe obj (priority,(r,c)):", 2*sys.getsizeof((999,999)))
#print("Size of int:", sys.getsizeof(1))

# Node size
NS = 2*sys.getsizeof((999,999))
TEST_SETUP = 1

    
g = Grid(120, 160, rand_state=0)

h_dict = {
    0: astar.h_function0,  # ucs
    1: astar.h_function1,
    2: astar.h_function2,
    3: astar.h_function3,
    4: astar.h_function4,
    5: astar.h_function5
}

'''
astar_search = astar.astar_search
print("grid#\tst/gl#\theur\tm_f(B)\texp\ttime(s)\tdist") # header
w = 1 # heuristic weight
for i in range(5):  # grid
    g = Grid(120, 160, rand_state=2*(i+1))

    for j in range(10):  # st/gl pairs
        for k in range(6):  # heuristic
            path, grids, test, p = astar_search(g, heuristic=h_dict[k], weight=w, test_stats=True)
            print("%d\t%d\t%d\t%d\t%d\t%3.3f\t%3.3f" % \
                (i, j, k, NS*test[0][p], NS*test[1][p], test[2], test[3]))

        g.replace_start_and_goal()

'''

'''
astar_search = astar.seq_astar_search

heurs = []
for i in range(1,6):
    heurs.append(h_dict[i])

print("w1\tw2\tgrid#\tst/gl#\theur\tm_f(B)\texp\ttime(s)\tdist") # header

w1 = 1
w2 = 1
weights = [1, 1.5, 3]
for a in range(3):
    for b in range(3):
        w1 = weights[a]
        w2 = weights[b]

        for i in range(5):  # grid
            g = Grid(120, 160, rand_state=2*(i+1))
        
            for j in range(10):  # st/gl pairs
        
        
                path, grids, test, p = astar_search(g, heurs, w1, w2, test_stats=True)
                mf = 0
                ex = 0
                for k in range(5):
                    mf += test[0][k]
                    ex += len(test[1][k])
                print("%d\t%d\t%d\t%d\t%d\t%d\t%d\t%3.3f\t%3.3f" % \
                    (a,b,i, j, k, NS*mf, NS*ex, test[2], test[3]))
        
                g.replace_start_and_goal()
'''



astar_search = astar.integ_astar_search

heurs = []
for i in range(1,6):
    heurs.append(h_dict[i])

print("w1\tw2\tgrid#\tst/gl#\theur\tm_f(B)\texp\ttime(s)\tdist") # header

w1 = 1
w2 = 1
weights = [1, 1.5, 3]
for a in range(3):
    for b in range(3):
        w1 = weights[a]
        w2 = weights[b]

        for i in range(5):  # grid
            g = Grid(120, 160, rand_state=2*(i+1))
        
            for j in range(10):  # st/gl pairs
        
        
                path, grids, test, p = astar_search(g, heurs, w1, w2, test_stats=True)
                mf = 0
                ex = 0
                for k in range(2):
                    mf += test[0][k]
                    ex += len(test[1][k])
                print("%d\t%d\t%d\t%d\t%d\t%d\t%d\t%3.3f\t%3.3f" % \
                    (a,b,i, j, k, NS*mf, NS*ex, test[2], test[3]))
        
                g.replace_start_and_goal()
