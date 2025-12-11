from collections import namedtuple
from typing import Any, Callable, Dict, List, Tuple

import pyomo.environ as pyo
from rust_enum import Case, enum 

"""
Example case with:
- single rectangle constraints (can only use each rectangle once)
- rectangles cannot overlap 
- maximize the value of each rectangle as objective

The space:  (3x4)
 _ _ _   
|_|_|_|
|_|_|_|
|_|_|_|
|_|_|_|

          _
rect_1 = |_| with value 1

          _
         |_|
rect_2 = |_| with value 2
         |_|

          _ _
         |_|_|
rect_3 = |_|_| with value 6
         |_|_|

          _ _
rect_4 = |_|_| with value 12
         |_|_|

The solution must be along these lines:
 _ _ _   
|1|_|_|
|2|_|_|
|2|4|4|
|2|4|4|

with total value: 1 + 2 + 12 = 15
"""

Rectangle = namedtuple("Rectangle", ["width", "height", "value"])
# space width & height
W, H = 3, 4

# name: (width, height, value)
rectangles = {
    "rect_1": Rectangle(1, 1, 1),
    "rect_2": Rectangle(1, 3, 2),
    "rect_3": Rectangle(2, 3, 6),
    "rect_4": Rectangle(2, 2, 12),
}

ConstraintF = Callable[[pyo.ConcreteModel], None]

def max_copy_constraint(max_copies: Dict[str, int])->ConstraintF:
    def constraint(model:pyo.ConcreteModel):
        model.max_copies = pyo.Param(model.R, initialize=max_copies)
        def max_copies_rule(m, r):
            return sum(m.place[r, x, y] for (x, y) in m.P_by_rect[r]) <= m.max_copies[r]

        model.limit_copies = pyo.Constraint(model.R, rule=max_copies_rule)
    return constraint

def single_rect_placement_constraint()-> ConstraintF:
    # At most one placement per rectangle
    def constraint(model:pyo.ConcreteModel):
        def one_placement_per_rect_rule(m, r):
            return sum(m.place[r, x, y] for (x, y) in m.P_by_rect[r]) <= 1

        model.one_placement_per_rect = pyo.Constraint(model.R, rule=one_placement_per_rect_rule)
    return constraint

def no_overlap_constraint()->ConstraintF:
    # No overlap on each grid cell
    def constraint(model:pyo.ConcreteModel):
        # Compute which placements cover a given cell (i,j)
        cell_to_placements = {(i, j): [] for i in range(W) for j in range(H)}
        for (r, x, y) in model.P:
            rw = model.w[r]
            rh = model.h[r]
            for i in range(x, x + rw):
                for j in range(y, y + rh):
                    cell_to_placements[(i, j)].append((r, x, y))

        model.C = pyo.Set(dimen=2, initialize=[(i, j) for i in range(W) for j in range(H)])
        def no_overlap_rule(m, i, j):
            return sum(m.place[r, x, y] for (r, x, y) in cell_to_placements[(i, j)]) <= 1

        model.no_overlap = pyo.Constraint(model.C, rule=no_overlap_rule)
    return constraint

@enum
class ObjectiveMode():
    MaxValue= Case()
    MaxAmount= Case()
    # Coefficient for value are deduced : 1-coeff_amount
    WeightedBoth = Case(coeff_amount=float)

def define_objective(model, mode:ObjectiveMode=ObjectiveMode.MaxValue):
    total_value = sum(
        model.v[r] * model.place[r, x, y]
        for (r, x, y) in model.P
    )
    total_amount = sum(model.place[p] for p in model.P)

    obj = None
    match mode: 
        case mode.MaxValue:
            obj = total_value
        case mode.MaxAmount:
            obj = total_amount
        case mode.WeightedBoth(coeff_amount):
            if coeff_amount<= 0.0:
                obj = total_value
            elif coeff_amount >= 1.0:
                obj = total_amount
            else:
                coeff_value = 1 - coeff_amount
                obj = coeff_amount * total_amount + coeff_value * total_value
        case _:
            raise ValueError(f"Unexpected mode: {mode}")

    model.obj = pyo.Objective(expr=obj, sense=pyo.maximize)

def solve(W:int,
          H:int,
          rectangles: Dict[str,Rectangle],
          objective: ObjectiveMode=ObjectiveMode.MaxValue,
          constraints : List[ConstraintF]= [single_rect_placement_constraint(), no_overlap_constraint()]
          )->Tuple[Any, pyo.ConcreteModel]:
    model = pyo.ConcreteModel()

    model.R = pyo.Set(initialize=list(rectangles.keys()))  # rectangles
    model.I = pyo.RangeSet(0, W - 1)  # x coordinates of cells
    model.J = pyo.RangeSet(0, H - 1)  # y coordinates of cells

    model.w = pyo.Param(model.R, initialize={r: rectangles[r].width for r in rectangles})
    model.h = pyo.Param(model.R, initialize={r: rectangles[r].height for r in rectangles})
    model.v = pyo.Param(model.R, initialize={r: rectangles[r].value for r in rectangles})

    # Precompute all feasible placements: (r, x, y)
    placements = []
    # Separate each by rectangle as a key for fast traversal
    placements_by_rect = {r: [] for r in model.R}

    for r in model.R:
        rw = model.w[r]
        rh = model.h[r]
        for x in range(W - rw + 1):
            for y in range(H - rh + 1):
                placements.append((r, x, y))
                placements_by_rect[r].append((x, y))

    model.P = pyo.Set(dimen=3, initialize=placements)  # P = {(r,x,y)}

    # Set of positions (x,y) for each r
    model.P_by_rect = pyo.Set(
        model.R,
        dimen=2,
        initialize=lambda m, r: placements_by_rect[r],
    )

    # Decision variables: place[r,x,y] = 1 if we place r with bottom-left at (x,y)
    model.place = pyo.Var(model.P, domain=pyo.Binary)

    for constraint in constraints:
        constraint(model)

    define_objective(model, objective)

    solver = pyo.SolverFactory("glpk")
    results = solver.solve(model, tee=True)
    return (results, model)

