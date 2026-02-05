
from __future__ import annotations
from typing import Any, Dict, Tuple, List
import heapq, math
from Algorithms.base import BaseAlgorithm

class AStarFuel(BaseAlgorithm):

    name = "AStarFuel"

    def __init__(self, fuel_step: float = 1.0):
        self.fuel_step = fuel_step

    def _discretize(self, x: float) -> float:
        return round(x / self.fuel_step) * self.fuel_step

    def solve(self, graph, start: str, goal: str, vehicle, weights: Dict[str, float], positions=None) -> Dict[str, Any]:
        if positions is None:
            raise ValueError("A* requires 'positions' for the heuristic.")
        w_dist = float(weights.get("distance", 1.0))
        w_fuel = float(weights.get("fuel", 1.0))

        cap = vehicle.tank_capacity
        cons = vehicle.consumption_per_dist

        def h(n: str) -> float:
            # lower bound: distance-only scaled by w_dist, no fuel-price guess
            return w_dist * math.hypot(positions[n][0]-positions[goal][0],
                                       positions[n][1]-positions[goal][1])

        pq: List[Tuple[float, float, str, float]] = []
        init_fuel = max(0.0, min(cap, vehicle.fuel))
        init_fuel = self._discretize(init_fuel)
        g0 = 0.0
        heapq.heappush(pq, (g0 + h(start), g0, start, init_fuel))

        # store[(node,fuel)] = (g_cost, total_distance, fuel_cost, parent_state, action)
        store: Dict[Tuple[str, float], Tuple[float, float, float, Tuple[str,float]|None, str|None]] = {}
        store[(start, init_fuel)] = (0.0, 0.0, 0.0, None, None)
        expanded = 0

        while pq:
            fscore, gcost, node, fuel_amt = heapq.heappop(pq)
            rec = store.get((node, fuel_amt))
            if rec is None or rec[0] < gcost - 1e-9:
                continue

            expanded += 1
            if node == goal:
                break

            # BUY fuel
            if fuel_amt + self.fuel_step <= cap:
                new_fuel = self._discretize(fuel_amt + self.fuel_step)
                price = graph.fuel_price(node) * self.fuel_step
                new_g = gcost + (w_fuel * price)
                cand = (new_g, rec[1], rec[2] + price, (node, fuel_amt), f"BUY {self.fuel_step}")
                key = (node, new_fuel)
                if key not in store or new_g < store[key][0] - 1e-9:
                    store[key] = cand
                    heapq.heappush(pq, (new_g + h(node), new_g, node, new_fuel))

            # MOVE
            for e in graph.neighbors(node):
                need = self._discretize(e.distance * cons)
                if fuel_amt + 1e-9 >= need:
                    new_node = e.to
                    new_fuel = self._discretize(fuel_amt - need)
                    add_dist = e.distance
                    new_g = gcost + (w_dist * add_dist)
                    cand = (new_g, rec[1] + add_dist, rec[2], (node, fuel_amt), f"GO {new_node}")
                    key = (new_node, new_fuel)
                    if key not in store or new_g < store[key][0] - 1e-9:
                        store[key] = cand
                        heapq.heappush(pq, (new_g + h(new_node), new_g, new_node, new_fuel))

        # best goal state
        best_key = None
        best_val = None
        for (n, f), val in store.items():
            if n == goal:
                if best_val is None or val[0] < best_val[0] - 1e-9:
                    best_key = (n, f)
                    best_val = val

        if best_key is None:
            return {"path": [], "total_distance": float("inf"), "fuel_cost": float("inf"),
                    "objective": float("inf"), "expanded": expanded, "notes": "No feasible path."}

        # reconstruct
        path_nodes: List[str] = []
        actions: List[str] = []
        cur = best_key
        while cur is not None:
            node, fuel = cur
            path_nodes.append(node)
            prev = store[cur][3]
            act = store[cur][4]
            if act is not None:
                actions.append(act)
            cur = prev
        path_nodes.reverse()
        actions.reverse()
        return {
            "path": path_nodes,
            "total_distance": store[best_key][1],
            "fuel_cost": store[best_key][2],
            "objective": store[best_key][0],
            "expanded": expanded,
            "notes": " | ".join(actions[:12]) + (" ..." if len(actions) > 12 else "")
        }
