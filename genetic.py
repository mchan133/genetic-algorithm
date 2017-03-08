import copy
import random
import time
import sys


DEBUG = False

goal = "1010101010101010101010101010101010101010101010101010101010101010"  # unknown to algorithm
GIVE_UP = 10000  # give up after x iterations
POOL_SIZE = 10


NUM_VARS = len(goal)
history = []  # a record of states


def find_solution(rand_state=None):
    if rand_state != None:
        random.seed(rand_state)

    gene_pool = ["" for i in range(POOL_SIZE)]
    initialize_states(gene_pool, sys.argv)  # random starting states
    
    counter = 0
    while counter < GIVE_UP:
        new_pool = []

        evaluations = eval_pool(gene_pool)

        add_elite(gene_pool, new_pool, evaluations)
        select(gene_pool, new_pool, evaluations)

        crossover(2, new_pool)
        mutate(2, new_pool)

        new_vals = eval_pool(new_pool)
        flip_heuristic2(2, new_pool, new_vals)

        #record(gene_pool)  # records chnges in pool to history

        if False and DEBUG:
            print(counter, "old:",gene_pool, evaluations)
            print(counter, "new:",new_pool, new_vals)
        gene_pool = new_pool

        if evaluate(gene_pool[0]) == NUM_VARS:
            print(">> PROBLEM SATISFIED at iteration " + str(counter))
            print(">> With solution:", gene_pool[0])
            print(">> Satisfied (" + str(new_vals[0]) + "/" + str(NUM_VARS) +") clauses.")
            sys.exit(0)

        counter += 1

    print(">> GAVE UP after " + str(GIVE_UP) + " tries.")
    print(">> Current Best:", gene_pool[0])
    print(">> Satisfied (" + str(new_vals[0]) + "/" + str(NUM_VARS) +") clauses.")


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


def initialize_states(gene_pool, argv=None):

    if argv:  #TODO: add support for user input states

        for i in range(POOL_SIZE):
            gene_pool[i] = create_state()

    else:
        for i in range(POOL_SIZE):
            gene_pool[i] = create_state()


def create_state():
    state = ""
    for i in range(NUM_VARS):
        state += str(int(flip_coin()))

    return state


def evaluate(state):
    result = 0
    for i in range(NUM_VARS):
        if state[i] == goal[i]:
            result += 1

    return result


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
        if evaluations[i] >= b1_score:  # adds overhead, but shuffles around ties
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

        while improvement:  # keeps going as long as there is improvement
            improvement  = False
            random.shuffle(order)

            for j in order:
                new_str, new_eval = eval_flip(new_pool[i], j, evaluations[i])
                if new_str:  # eval flip returns None if not better
                    new_pool[i] = new_str
                    evaluations[i] = new_eval
                    improvement = True

    return None

def eval_flip(string, index, evaluation):
    # flipping bit at index i
    new_str = string[:index] + ("1" if string[index]=="0" else "0") + string[(index+1):]

    new_eval = evaluate(new_str)
    if new_eval >= evaluation:
        return (new_str, new_eval)
     
    return (None, None)


'''
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
'''


def read_cnf(file):



    return (None)







if __name__ == '__main__':

    find_solution(rand_state=0)


