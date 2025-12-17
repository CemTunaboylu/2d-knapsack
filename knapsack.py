from collections import namedtuple
from itertools import product
from typing import Any, Callable, Dict, List, Set, Tuple

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

def inject_cell_to_placements_in_model_if_not_in(model, cells_key:str= "Cells", cover_key:str= "Cover", cover_by_cell_key:str="CoverByCell") :
    if getattr(model, cover_key, None) and getattr(model, cover_by_cell_key, None):
        return

    cells = set()
    cover_by_cell = dict()
    cover_tuples = []
    for w,h in product(range(model.W), range(model.H)):
        cells.add((w,h))
        cover_by_cell[(w,h)] = []

    for (r, x, y) in model.Placements:
        rw = model.w[r]
        rh = model.h[r]
        for i in range(x, x + rw):
            for j in range(y, y + rh):
                cover_tuples.append((i, j, r, x, y))
                cover_by_cell[(i, j)].append((r, x, y))

    cells = pyo.Set(dimen=2, initialize=cells)
    setattr(model, cells_key, cells)
    cover = pyo.Set(dimen=5, initialize=cover_tuples)
    setattr(model, cover_key, cover)

    cover_by_cell_set = pyo.Set(
        model.Cells, dimen=3,
        initialize=lambda _m, i, j: cover_by_cell[(i, j)]
    )

    setattr(model, cover_by_cell_key, cover_by_cell_set)


ConstraintF = Callable[[pyo.ConcreteModel], None]

def obstacle_cells_constraint(blocked_cells: Set[Tuple[int,int]], cells_key:str = "Cells", cover_by_cell_key:str= "CoverByCell") -> ConstraintF:
    def constraint(model: pyo.ConcreteModel):
        inject_cell_to_placements_in_model_if_not_in(model)
        model.Blocked = pyo.Set(dimen=2, initialize=list(blocked_cells))

        def rule(m, i, j):
            #  for blocked cells: <= 0, otherwise do nothing (Skip)
            if (i,j) not in model.Blocked:
                return pyo.Constraint.Skip

            cover_by_cell = getattr(m, cover_by_cell_key)
            if len(cover_by_cell[i,j]) == 0:
                return pyo.Constraint.Feasible

            cover_by_cell = getattr(m, cover_by_cell_key)
            return sum(m.place[r, x, y] for (r, x, y) in cover_by_cell[i, j]) <= 0

        model.obstacles = pyo.Constraint(model.Cells, rule=rule)
    return constraint

def max_copy_constraint(max_copies: Dict[str, int])->ConstraintF:
    def constraint(model:pyo.ConcreteModel):
        model.max_copies = pyo.Param(model.Rectangles, initialize=max_copies)
        def max_copies_rule(m, r):
            return sum(m.place[r, x, y] for (x, y) in m.P_by_rect[r]) <= m.max_copies[r]

        model.limit_copies = pyo.Constraint(model.Rectangles, rule=max_copies_rule)
    return constraint

def single_rect_placement_constraint()-> ConstraintF:
    # At most one placement per rectangle
    def constraint(model:pyo.ConcreteModel):
        def one_placement_per_rect_rule(m, r):
            return sum(m.place[r, x, y] for (x, y) in m.P_by_rect[r]) <= 1

        model.one_placement_per_rect = pyo.Constraint(model.Rectangles, rule=one_placement_per_rect_rule)
    return constraint

def no_overlap_constraint(cells_key:str = "Cells", cover_by_cell_key:str= "CoverByCell")->ConstraintF:
    def constraint(model:pyo.ConcreteModel):
        inject_cell_to_placements_in_model_if_not_in(model)
        def rule(m, i, j):
            cover_by_cell = getattr(m, cover_by_cell_key)
            if len(cover_by_cell[i,j]) == 0: return pyo.Constraint.Skip

            return sum(m.place[r, x, y] for (r, x, y) in m.CoverByCell[i, j]) <= 1

        cells = getattr(model, cells_key)
        model.no_overlap = pyo.Constraint(cells, rule=rule)
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
        for (r, x, y) in model.Placements
    )
    total_amount = sum(model.place[p] for p in model.Placements)

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

    model.Rectangles = pyo.Set(initialize=list(rectangles.keys()))  # rectangles
    model.W, model.H = W, H

    model.w = pyo.Param(model.Rectangles, initialize={r: rectangles[r].width for r in rectangles})
    model.h = pyo.Param(model.Rectangles, initialize={r: rectangles[r].height for r in rectangles})
    model.v = pyo.Param(model.Rectangles, initialize={r: rectangles[r].value for r in rectangles})

    # Precompute all feasible placements: (r, x, y)
    placements = []
    # Separate each by rectangle as a key for fast traversal
    placements_by_rect = {r: [] for r in model.Rectangles}

    for r in model.Rectangles:
        rw = model.w[r]
        rh = model.h[r]
        for x in range(model.W - rw + 1):
            for y in range(model.H - rh + 1):
                placements.append((r, x, y))
                placements_by_rect[r].append((x, y))

    model.Placements = pyo.Set(dimen=3, initialize=placements)  # P = {(r,x,y)}

    # Set of positions (x,y) for each r
    model.P_by_rect = pyo.Set(
        model.Rectangles,
        dimen=2,
        initialize=lambda _m, r: placements_by_rect[r],
    )

    # Decision variables: place[r,x,y] = 1 if we place r with bottom-left at (x,y)
    model.place = pyo.Var(model.Placements, domain=pyo.Binary)

    for constraint in constraints:
        constraint(model)

    define_objective(model, objective)

    solver = pyo.SolverFactory("glpk")
    results = solver.solve(model, tee=True)
    return (results, model)

