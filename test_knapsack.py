import unittest

from knapsack import *

class TestKnapsack(unittest.TestCase):
    def test_4_rects_max_value(self): 
        W, H = 3, 4

        rectangles = {
            "rect_1": Rectangle(1, 1, 1),
            "rect_2": Rectangle(1, 3, 2),
            "rect_3": Rectangle(2, 3, 6),
            "rect_4": Rectangle(2, 2, 12),
        }

        results, model = solve(W, H, rectangles)
        print(results.solver.status, results.solver.termination_condition)

        opt_val = pyo.value(model.obj)
        print("Optimal value:", opt_val)

        # Assert that it matches 15 (up to small numerical tolerance)
        exp = 15.0
        self.assertTrue(abs(opt_val - exp) < 1e-6, f"Expected {exp}, got {opt_val}")

        for (r, x, y) in model.P:
            if pyo.value(model.place[r, x, y]) > 0.5:
                print(f"Place {r} at (x={x}, y={y})")

    def test_3_rects_max_amount_max_copies(self): 
        W, H = 3, 4

        rectangles = {
            "single": Rectangle(1, 1, 1),
            "double": Rectangle(1, 3, 2),
            "triple": Rectangle(2, 3, 6),
        }

        max_copies = {"single": 10, "double": 10, "triple": 10}
        
        constraints = [
            max_copy_constraint(max_copies),
            no_overlap_constraint() ]

        results, model = solve(W, H, rectangles, ObjectiveMode.MaxAmount, constraints)
        print(results.solver.status, results.solver.termination_condition)

        opt_val = pyo.value(model.obj)
        print("Optimal value:", opt_val)

        # Assert that it matches 10 (up to small numerical tolerance)
        exp = 10.0
        self.assertTrue(abs(opt_val - exp) < 1e-6, f"Expected {exp}, got {opt_val}")

        for (r, x, y) in model.P:
            if pyo.value(model.place[r, x, y]) > 0.5:
                print(f"Place {r} at (x={x}, y={y})")

    def test_4_rects_max_value_max_copies(self): 
        W, H = 3, 4

        rectangles = {
            "single": Rectangle(1, 1, 1),
            "double": Rectangle(1, 3, 2),
            "triple": Rectangle(2, 3, 4),
        }

        
        max_copies = {"single": 10, "double": 10, "triple": 10}
        
        constraints = [
            max_copy_constraint(max_copies),
            no_overlap_constraint() ]

        results, model = solve(W, H, rectangles, ObjectiveMode.MaxValue, constraints)
        print(results.solver.status, results.solver.termination_condition)

        opt_val = pyo.value(model.obj)
        print("Optimal value:", opt_val)

        # Assert that it matches 11 (up to small numerical tolerance)
        exp = 11.0
        self.assertTrue(abs(opt_val - exp) < 1e-6, f"Expected {exp}, got {opt_val}")

        for (r, x, y) in model.P:
            if pyo.value(model.place[r, x, y]) > 0.5:
                print(f"Place {r} at (x={x}, y={y})")
