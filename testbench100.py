import time
import sys
import genetic

if __name__ == "__main__":
    tt = 3
    test_folder = ["uf20-91/", "uf50-218/", "uf75-325/", "uf100-430/"] 
    name_prefix = ["uf20-0", "uf50-0", "uf75-0", "uf100-0"]
    name_suffix = ".cnf"

    # 20, 50, 75, 100
    give_up = [500, 1000, 1500, 2000]

    for i in range(100):
        filename = test_folder[tt] + name_prefix[tt] + str(i+1) + name_suffix
        variables, cnf = genetic.read_cnf(filename)

        start_t = time.clock()
        success, flips = genetic.find_solution(variables, cnf, start_states=None, rand_state=0, rand_restarts=True, tries=give_up[tt], tb=True)
        stop_t = time.clock()

        # tt, i, success, time
        print("%d\t%d\t%d\t%d\t%f" % (tt, i, success, flips, (stop_t-start_t)))

