# 8-Puzzle Solver

## Overview

This repository contains Python code to solve the 8-puzzle problem using various search algorithms. The 8-puzzle problem is a classic problem in artificial intelligence, where the goal is to rearrange a 3x3 grid of numbered tiles to reach a specified goal configuration.

## Features

- Breadth-First Search (BFS)
- Depth-First Search (DFS)
- Uniform Cost Search (UCS)
- Greedy Search
- A* Search

## Getting Started

### Prerequisites

- Python 3.x
- `numpy` library
- `scipy` library
- `sympy` library

Install the required libraries using:

```bash
pip install numpy scipy sympy
```

### Usage

Run the solver by executing the following command in the terminal:

```bash
python 8_puzzle_solver.py <start_state_file> <goal_state_file> <method> <dumpflag>
```

- `<start_state_file>`: Path to the file containing the initial state of the puzzle.
- `<goal_state_file>`: Path to the file containing the goal state of the puzzle.
- `<method>`: Search method (`bfs`, `dfs`, `ucs`, `greedy`, `a*`).
- `<dumpflag>`: Dump trace information (`true` or `false`).

### Example

```bash
python 8_puzzle_solver.py initial_state.txt goal_state.txt astar true
```

## File Formats

### Puzzle State Files

The puzzle state files should be text files containing a 3x3 grid of numbers. Each row should have exactly 3 numbers, and there should be 3 rows in total. For example:

```
1 2 3
4 5 6
7 8 0
```

Here, `0` represents the empty tile.

### Output Trace File

If `dumpflag` is set to `true`, the program will generate a trace file named `trace-dd-mm-yyyy-HH-MM-SS.txt` containing detailed information about the execution, including state transitions, actions, costs, and more.

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.
