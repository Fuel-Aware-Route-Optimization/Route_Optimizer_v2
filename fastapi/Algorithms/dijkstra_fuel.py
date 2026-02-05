
from __future__ import annotations
from typing import Any, Dict, Tuple, List
import heapq
from Algorithms.base import BaseAlgorithm

class DijkstraFuel(BaseAlgorithm):

    name = "DijkstraFuel"

    def __init__(self, fuel_step: float = 1.0):
        self.fuel_step = fuel_step

    def _discretize(self, x: float) -> float:
        # round to nearest multiple of fuel_step
        return round(x / self.fuel_step) * self.fuel_step

    def solve(self, graph, start: str, goal: str, vehicle, weights: Dict[str, float], positions=None) -> Dict[str, Any]:
        w_dist = float(weights.get("distance", 1.0))
        w_fuel = float(weights.get("fuel", 1.0))

        cap = vehicle.tank_capacity
        cons = vehicle.consumption_per_dist

        # Priority queue on blended cost so far
        pq: List[Tuple[float, str, float]] = []
        init_fuel = max(0.0, min(cap, vehicle.fuel))
        init_fuel = self._discretize(init_fuel)
        heapq.heappush(pq, (0.0, start, init_fuel))

        # dist[(node, fuel)] = (blended_cost, total_distance, fuel_cost, parent_state, action)
        dist: Dict[Tuple[str, float], Tuple[float, float, float, Tuple[str,float]|None, str|None]] = {}
        dist[(start, init_fuel)] = (0.0, 0.0, 0.0, None, None)

        expanded = 0

        while pq:
            cost_so_far, node, fuel_amt = heapq.heappop(pq)
            store = dist.get((node, fuel_amt))
            if store is None or store[0] < cost_so_far - 1e-9:
                continue  # stale

            expanded += 1
            if node == goal:
                # reconstruct best among goal states with varying fuel
                break

            # Option 1: Buy one step of fuel (if capacity allows)
            if fuel_amt + self.fuel_step <= cap:
                new_fuel = self._discretize(fuel_amt + self.fuel_step)
                price = graph.fuel_price(node) * self.fuel_step
                new_blended = cost_so_far + (w_fuel * price)
                prev_total_dist = store[1]
                prev_fuel_cost = store[2]
                cand = (new_blended, prev_total_dist, prev_fuel_cost + price, (node, fuel_amt), f"BUY {self.fuel_step}")
                key = (node, new_fuel)
                if key not in dist or cand[0] < dist[key][0] - 1e-9:
                    dist[key] = cand
                    heapq.heappush(pq, (new_blended, node, new_fuel))

            # Option 2: Traverse edges if enough fuel
            for e in graph.neighbors(node):
                needed = e.distance * cons
                needed = self._discretize(needed)
                if fuel_amt + 1e-9 >= needed:
                    new_fuel = self._discretize(fuel_amt - needed)
                    new_node = e.to
                    add_dist = e.distance
                    new_blended = cost_so_far + (w_dist * add_dist)
                    prev_total_dist = store[1]
                    prev_fuel_cost = store[2]
                    cand = (new_blended, prev_total_dist + add_dist, prev_fuel_cost, (node, fuel_amt), f"GO {new_node}")
                    key = (new_node, new_fuel)
                    if key not in dist or cand[0] < dist[key][0] - 1e-9:
                        dist[key] = cand
                        heapq.heappush(pq, (new_blended, new_node, new_fuel))

        # Choose best goal state among all fuel levels
        best_key = None
        best_val = None
        for (n, f), val in dist.items():
            if n == goal:
                if best_val is None or val[0] < best_val[0] - 1e-9:
                    best_key = (n, f)
                    best_val = val

        if best_key is None:
            return {"path": [], "total_distance": float("inf"), "fuel_cost": float("inf"),
                    "objective": float("inf"), "expanded": expanded, "notes": "No feasible path."}

        # Reconstruct path of nodes (ignore BUY actions in path, but we keep notes)
        path_nodes: List[str] = []
        actions: List[str] = []
        cur = best_key
        while cur is not None:
            node, fuel = cur
            path_nodes.append(node)
            prev = dist[cur][3]
            act = dist[cur][4]
            if act is not None:
                actions.append(act)
            cur = prev
        path_nodes.reverse()
        actions.reverse()

        return {
            "path": path_nodes,
            "total_distance": dist[best_key][1],
            "fuel_cost": dist[best_key][2],
            "objective": dist[best_key][0],
            "expanded": expanded,
            "notes": " | ".join(actions[:12]) + (" ..." if len(actions) > 12 else "")
        }
