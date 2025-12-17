2D Knapsack (Rectangle Packing) with Pyomo

A tiny, hackable MILP model for packing axis-aligned rectangles into a discrete 2D grid using Pyomo + GLPK. Supports:

- No overlap packing constraints
- Optional max copies per rectangle type
- Objectives:
- maximize total value
- maximize number of rectangles placed
- weighted combination of both

Repo Layout
```
2d-knapsack/
  knapsack.py
  requirements.txt
  test_knapsack.py
```

## Problem

Given a grid of size W × H and a set of rectangles (width, height, value), choose placements (x,y) (bottom-left corner) such that rectangles do not overlap. Optimize an objective like:
-	MaxValue: maximize sum of rectangle values
-	MaxAmount: maximize number of placed rectangles
-	WeightedBoth: trade off value vs count

The included demo instance has an optimum total value of 15 and the unit test asserts it.

## Install

```sh
python -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

Dependencies are pinned in requirements.txt (Pyomo + ply).

You also need GLPK installed on your system so Pyomo can call it:
-	macOS: `brew install glpk`
-	Ubuntu: `sudo apt-get install glpk-utils`

## Quick Start

Solve a packing instance

```py
from knapsack import Rectangle, solve, ObjectiveMode, no_overlap_constraint, single_rect_placement_constraint

W, H = 3, 4
rectangles = {
    "rect_1": Rectangle(1, 1, 1),
    "rect_2": Rectangle(1, 3, 2),
    "rect_3": Rectangle(2, 3, 6),
    "rect_4": Rectangle(2, 2, 12),
}

constraints = [
    single_rect_placement_constraint(),  # use each rectangle at most once  
    no_overlap_constraint(),             # prevent overlap per cell  
]

results, model = solve(W, H, rectangles, objective=ObjectiveMode.MaxValue, constraints=constraints)
print("Objective:", model.obj())

````
## Objectives

Available objective modes are defined in ObjectiveMode  ￼:
- `ObjectiveMode.MaxValue` — maximize total value
- `ObjectiveMode.MaxAmount` — maximize number of placements
- `ObjectiveMode.WeightedBoth(coeff_amount=...)` — weighted blend of count and value  ￼

## Example:

```py
solve(W, H, rectangles, objective=ObjectiveMode.WeightedBoth(coeff_amount=0.3))
```

## Constraints

### No overlap

Ensures each grid cell is covered by at most one placed rectangle.

### Single-use rectangles

Use each rectangle at most once (classic 0/1 knapsack-style).

### Bounded copies (repeatable room types)

Limit copies per rectangle type with max_copy_constraint:

```py
max_copies = {"single": 10, "double": 10, "triple": 10}
constraints = [max_copy_constraint(max_copies), no_overlap_constraint()]
```

### Obstacles

To model walls/columns/unusable floor area, define blocked cells and prevent any rectangle from covering them.

Two common approaches:
	1.	Cell constraint (RHS=0): for blocked cells (i,j), enforce: `sum(placements covering (i,j)) <= 0`. This is the approach taken here.
	2.	Fix placements: if a placement overlaps any blocked cell, set: `place[r,x,y].fix(0)`

Both approaches integrate cleanly with the existing “placements cover cells” structure used for overlap constraints.

**Note**: Our approach 1. treats obstacles as a set of blocked grid cells B ⊆ {(i,j)}. In the overlap constraint, we make those cells unusable:
- For free cells: `sum(covering placements) <= 1`
- For blocked cells: `sum(covering placements) <= 0` *That means “nothing may cover this cell”.*



## Tests

Run:

`python -m unittest -v`

There is a test that asserts the demo instance reaches objective value 15.
