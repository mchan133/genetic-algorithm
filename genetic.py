import copy
import random
import time
import sys


# TODO: accept user args for states, give up, etc
# TODO: set up signal handler for Ctrl-C
DEBUG = True

history = []  # a record of states
GIVE_UP = 1000  # give up after x iterations
POOL_SIZE = 10

NUM_VARS = 0
NUM_CLAUSES = 0
CNF = [] #TODO: fix evaluate function


def find_solution(variables, cnf, start_states=None, rand_state=None, rand_restarts=False, tries=None, tb=False):

    global NUM_VARS
    global NUM_CLAUSES
    global CNF
    global GIVE_UP

    NUM_VARS = variables
    NUM_CLAUSES = len(cnf)
    CNF = cnf
    #GIVE_UP = NUM_VARS**2  # 20->400, 50->2500, 100->10k
    GiVE_UP = tries

    if not tries:
        tries = 5*NUM_VARS

    if rand_state != None:
        random.seed(rand_state)

    gene_pool = ["" for i in range(POOL_SIZE)]
    initialize_states(gene_pool, start_states)  # random starting states
    if DEBUG: print(gene_pool)
    
    counter = 0
    restart = int(tries/5)
    flips = 0
    while counter < GIVE_UP:
        if rand_restarts and counter>0 and not (counter % restart):  # random restarts
            if not tb: print("restarting: (" + str(new_vals[0]) + "/" + str(NUM_CLAUSES) + ")")
            initialize_states(gene_pool, gene_pool[0])

        if not tb: print("iteration", counter, "\t\t", gene_pool[0])
        new_pool = []

        evaluations = eval_pool(gene_pool)

        elite_states = add_elite(gene_pool, new_pool, evaluations)
        select(gene_pool, new_pool, evaluations)

        crossover(2, new_pool)
        mutate(2, new_pool)

        new_vals = eval_pool(new_pool)
        flips += flip_heuristic2(2, new_pool, new_vals)

        #record(gene_pool)  # records chnges in pool to history

        if False and DEBUG:
            print(counter, "old:",gene_pool, evaluations)
            print(counter, "new:",new_pool, new_vals)
        gene_pool = new_pool

        if evaluate(gene_pool[0]) == NUM_CLAUSES:
            if not tb:  # test bench
                print(">> PROBLEM SATISFIED at iteration " + str(counter))
                print(">> With solution:", readable(gene_pool[0]))
                print(">> Satisfied (" + str(new_vals[0]) + "/" + str(NUM_CLAUSES) +") clauses.")
            return (0, flips)

        counter += 1

    if not tb:  # test bench
        print(">> GAVE UP after " + str(GIVE_UP) + " tries.")
        print(">> Current Best:", readable(gene_pool[0]))
        print(">> Satisfied (" + str(new_vals[0]) + "/" + str(NUM_CLAUSES) +") clauses.")
        print("UNSATISFIED CLAUSES: (1-indexed)")
        for i in range(len(CNF)):
            if not satisfied(CNF[i], gene_pool[0]):
                print(str(i+1) + ":\t", CNF[i])
    return (1, flips)


def readable(string):
    new_str = ""

    for i in range(len(string)):
        if i%5 == 0: new_str += " "
        new_str += string[i]
    
    return new_str


def flip_coin(p=.5):
    if (p < 0 or p > 1):
        raise ValueError("p can only be between 0 and 1.")
    return int(random.random() < p)


def sample(probs, selections):
    samples = []
    cdf = []
    for i in range(len(probs)-1):
        cdf.append(sum(probs[:i+1]))
    cdf.append(1)

    while len(samples) < selections:
        p = random.random()
        i = 0
        while(p >= cdf[i]):
            i += 1
        samples.append(i)

    return samples


def print_pool(gene_pool):
    for i in range(len(gene_pool)):
        print(gene_pool[i])


def record(gene_pool):
    history.append(copy.deepcopy(gene_pool))


def initialize_states(gene_pool, given=None):

    if given and (len(given)<len(gene_pool)):
        for i in range(len(given)):
            gene_pool[i] = given[i]

        for i in range(len(given), POOL_SIZE):
            gene_pool[i] = create_state()

    else:
        for i in range(POOL_SIZE):
            gene_pool[i] = create_state()


def create_state():
    state = ""
    for i in range(NUM_VARS):
        state += str(int(flip_coin()))

    return state


#TODO: to be replaced with cnf evaluation function for sat problems
'''
def evaluate(state):
    #result = 0
    #for i in range(NUM_CLAUSES):
    #    if state[i] == goal[i]:
    #        result += 1

    #return result
    return cnf_eval(CNF, state)
'''



def eval_pool(pool):
    evaluations = []
    for i in range(len(pool)):
        evaluations.append(evaluate(pool[i]))

    return evaluations


def add_elite(old_pool, new_pool, evaluations):
    best1 = old_pool[0]
    b1_score = evaluations[0]
    best2 = best1  # keep the same for now
    b2_score = b1_score

    for i in range(1, POOL_SIZE):
        if evaluations[i] >= b1_score:  # shuffles around ties
            best2 = best1
            b2_score = b1_score

            best1 = old_pool[i]
            b1_score = evaluations[i]

    new_pool.append(best1)
    new_pool.append(best2)


def select(gene_pool, new_pool, evaluations):
    probs = [0 for i in range(POOL_SIZE)]
    selections = POOL_SIZE - len(new_pool)
    denom = sum(evaluations)

    for i in range(POOL_SIZE):
        probs[i] = evaluations[i]/denom

    result = sample(probs, selections)
    for i in range(selections):
        new_pool.append(gene_pool[result[i]])


def crossover(safe, new_pool):
    for i in range(safe, POOL_SIZE, 2):
        a, b = cross(new_pool[i], new_pool[i+1])
        new_pool[i]   = a
        new_pool[i+1] = b


def cross(x, y):
    guide = ""
    for i in range(len(x)):
        guide += str(int(flip_coin()))

    c1 = ""
    c2 = ""
    for i in range(len(x)):
        if guide[i] == "1":
            c1 += x[i]
            c2 += y[i]
        else:
            c1 += y[i]
            c2 += x[i]

    return (c1, c2)

def mutate(safe, new_pool):
    for i in range(safe, POOL_SIZE):
        if flip_coin(.9):
            mutant = ""

            for j in range(len(new_pool[i])):
                if flip_coin():
                    mutant += str(1 - int(new_pool[i][j]))
                else:
                    mutant += new_pool[i][j]

            new_pool[i] = mutant


# the actual flip heuristic (local changes)
# now, should instantly solve convex optimizations
def flip_heuristic2(safe, new_pool, evaluations):

    for i in range(safe, POOL_SIZE):
        improvement = True
        order = [j for j in range(NUM_VARS)]
        flips = 0

        while improvement:  # keeps going as long as there is improvement
            improvement  = False
            random.shuffle(order)

            for j in order:
                new_str, new_eval = eval_flip(new_pool[i], j, evaluations[i])
                flips +=1
                if new_str:  # eval flip returns None if not better
                    new_pool[i] = new_str
                    evaluations[i] = new_eval
                    #improvement = True

    return flips

def eval_flip(string, index, evaluation):
    # flipping bit at index i
    new_str = string[:index] + ("1" if string[index]=="0" else "0") + string[(index+1):]

    new_eval = evaluate(new_str)
    if new_eval > evaluation:
        return (new_str, new_eval)
     
    return (None, None)


# not currently using
def flip_heuristic(safe, new_pool, evaluations):
    for i in range(safe, POOL_SIZE):
        flipped = flip_bits(new_pool[i])
        value = evaluate(flipped)

        if value >= evaluations[i]:
            evaluations[i] = value
            new_pool[i] = flipped
def flip_bits(string):
    new_str = ""
    for i in range(NUM_VARS):
        new_str += "1" if string[i]=="0" else "0"
    return new_str


# right now, assuming that lines end with zero, one clause per line
def read_cnf(file_name):
    f = open(file_name, "r")

    lines = f.read().splitlines()
    variables = 0
    clauses = 0

    for i in range(len(lines)):
        if lines[i][0] == 'p':  # found problem line
            variables, clauses = map(int, lines[i].split()[2:])
            if DEBUG: print(variables, clauses)

            lines = lines[(i+1):]
            break;

    cnf = []
    for i in range(clauses):
        cnf.append(list(map(int, lines[i].split())))

    for clause in cnf: # removes 0 from the end (assumption)
        if clause[-1] == 0:
            clause.pop()

    return (variables, cnf)

def cnf_eval(cnf, state):
    sat_clauses = 0

    for i in range(len(cnf)):
        sat = satisfied(cnf[i], state)
        #if DEBUG: print(i, sat_clauses, "c:",cnf[i],"s:",state, sat)
        if sat:
            sat_clauses += 1

    return sat_clauses

def evaluate(state):
    sat_clauses = 0

    for i in range(NUM_CLAUSES):
        sat = satisfied(CNF[i], state)
        #if DEBUG: print(i, sat_clauses, "c:",cnf[i],"s:",state, sat)
        if sat:
            sat_clauses += 1

    return sat_clauses

# simple, doesn't tell you how satisfied the clause is (don't think that matters)
def satisfied(clause, state):
    #for i in range(len(clause)):
    for i in range(3):
        temp = -clause[i] if (clause[i] < 0) else clause[i]
        # (if the variable is true) != (if the variable is negated)
        truthy = (state[temp-1]=="1") != (temp==clause[i])
        if truthy:
            return True

    return False


######
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

def local_tsp(cities, adj):
    return

    




if __name__ == '__main__':

    import cProfile
    import pstats


    #variables, cnf = read_cnf("uf50-218/uf50-01.cnf")
    variables, cnf = read_cnf("uf100-430/uf100-01.cnf")

    pr = cProfile.Profile()
    pr.enable()

    #start_t = time.clock()
    find_solution(variables, cnf, start_states=None, rand_state=0, rand_restarts=True)
    #stop_t = time.clock()
    #print("TIME ELAPSED: " + str(stop_t - start_t) + " seconds.")

    pr.disable()
    pstats.Stats(pr).print_stats()


