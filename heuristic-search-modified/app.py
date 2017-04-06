#!/usr/bin/env python

# app.py
# Authors: Matthew Chan and Jeremy Savarin
# CS440: Project 1 - Heuristic Search

import sys

from heuristic_search.grid import run_search

'''
usage: python app.py {mode pairs}
'file'   [filename]         - generate grid from file
    default: False
'gen'    [int(seed)|'None'] - generate grid from seed (None for random)
    default: None
'heur'   [0-5]              - Use heuristic X (from 0 to 5)
    default: 0
'weight' [float >= 0]        - Weight the A* algorithm
    default: 1
'type'   ['seq'|'int']      - Type of algorithm
    default: regular A*
Only with the type arg:
    'heurs' [heuristic string] - First heuristic should be admissible
        default: '123'
        NOTE: minimal error checking, be careful with input args
    'w2'    [float >= 0]       - Bounding for inadmissible heuristics
        default: 1
'''


if __name__ == '__main__':
    filename = None
    rand_state = None
    mode = True  # true=gen, false=file
    heuristic = 0
    weight = 1.0
    s_type = 0
    heurs = [1,2,3]
    w2 = 1


    for i in range(len(sys.argv)):
        if i == (len(sys.argv) - 1):
            break

        if sys.argv[i] == 'file':
            filename = sys.argv[i+1]
            mode = False

        if sys.argv[i] == 'gen':
            if sys.argv[i+1] != 'None':
                rand_state = int(sys.argv[i+1])

        if sys.argv[i] == 'heur':
            if 0 <= int(sys.argv[i+1]) <= 5:  # valid heuristic
                heuristic = int(sys.argv[i+1])

        if sys.argv[i] == 'weight':
            if 0 <= float(sys.argv[i+1]):  # valid weight
                weight = float(sys.argv[i+1])

        if sys.argv[i] == 'type':
            s = sys.argv[i+1]
            if s == 'seq':
                s_type = 1
            elif s == 'int':
                s_type = 2

        # minimal error checking
        if sys.argv[i] == 'heurs':
            s = sys.argv[i+1]
            li = list(s)
            ls = []
            try:
                for j in li:
                    ls.append(int(j))
                heurs = ls
            except Exception:
                pass

        if sys.argv[i] == 'w2':
            if 0 <= float(sys.argv[i+1]):  # valid weight
                w2 = float(sys.argv[i+1])



    run_search(mode, filename, rand_state, heuristic, weight, s_type, heurs, w2)
