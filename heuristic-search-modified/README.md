# CS440: Project 1 - Heuristic Search

## About

This is the first project for CS440 - Intro to Artificial Intelligence. The goal is to evaluate the effect of different heuristic functions on three informed search algorithms: uniform-cost search, A\*, and weighted A\*.

## Installation

1. Clone the repo: `git clone https://github.com/JeremySavarin/heuristic-search.git`
2. Enter the directory: `cd heuristic-search`
3. Install dependencies: `pip install -r requirements.txt`

## Usage

Heuristic function codes:
* `0: Uniform-cost search`
* `1: Euclidean distance / 4`
* `2: Manhattan distance `
* `3: Square root of Manhattan distance`
* `4: Sum of squares`
* `5: Radial distance`

If the user does not specify a heuristic or weight, the program defaults to uniform-cost search with a weight of 1.

1. Load grid from text file: `python app.py file [filename]`
2. Generate random grid from seed or "None": `python app.py gen [seed|"None"]`

Other options:

1. Generate random grid and run weighted (w=1.5) A* with radial distance heuristic: `python app.py gen [seed] heur 5 weight 1.5`

Once the grid is generated, use the A key to toggle the path and visited nodes to display on the grid.

## Authors

Matthew Chan and Jeremy Savarin
