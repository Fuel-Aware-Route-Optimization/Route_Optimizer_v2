#!/usr/bin/env python3

# eval.py -- comparing A* heuristic variants & baselines.

# Run from the fastapi/ 
#     python -m Algorithms.eval


from __future__ import annotations
import sys, os, time, math, io

# Force UTF-8 output on Windows
if sys.platform == "win32":
    sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding="utf-8", errors="replace")

# ensure the fastapi dir is on sys.path so `Algorithms.*` imports work
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from services.graph import Graph, Edge, Vehicle, generate_random_graph
from Algorithms.dijkstra_fuel import DijkstraFuel
from Algorithms.greedy_cheap_fuel import GreedyCheapFuel
from Algorithms.astar_fuel import AStarFuel
from Algorithms.astar_fuel_optimized import AStarFuelOptimized



#  Pretty-printing helpers


CYAN   = "\033[96m"
GREEN  = "\033[92m"
YELLOW = "\033[93m"
RED    = "\033[91m"
BOLD   = "\033[1m"
DIM    = "\033[2m"
RESET  = "\033[0m"


def banner(text):
    w = 64
    print()
    print(f"{CYAN}{'=' * w}{RESET}")
    print(f"{CYAN}|{RESET} {BOLD}{text.center(w - 4)}{RESET} {CYAN}|{RESET}")
    print(f"{CYAN}{'=' * w}{RESET}")


def section(text):
    print(f"\n{YELLOW}-- {text} --{RESET}")


def fmt(v, precision=2):
    if v == float("inf"):
        return f"{RED}   INF{RESET}"
    return f"{v:>{7}.{precision}f}"


def print_table(rows):
    """Print a comparison table from a list of result dicts."""
    header = (
        f"  {'Algorithm':<22} {'Distance':>9} {'Fuel $':>9} "
        f"{'Objective':>10} {'Expanded':>9} {'Time ms':>8}"
    )
    print(f"{DIM}{header}{RESET}")
    print(f"{DIM}  {'-' * 22} {'-' * 9} {'-' * 9} {'-' * 10} {'-' * 9} {'-' * 8}{RESET}")

    # find best objective among feasible results
    feasible = [r for r in rows if r["objective"] < float("inf")]
    best_obj = min((r["objective"] for r in feasible), default=float("inf"))
    least_expanded = min((r["expanded"] for r in feasible), default=float("inf"))

    for r in rows:
        name = r["name"]
        obj = r["objective"]
        exp = r["expanded"]

        obj_color = GREEN if obj <= best_obj + 1e-9 else ""
        exp_color = GREEN if exp <= least_expanded else ""
        obj_reset = RESET if obj_color else ""
        exp_reset = RESET if exp_color else ""

        print(
            f"  {name:<22} {fmt(r['distance']):>9} {fmt(r['fuel_cost']):>9} "
            f"{obj_color}{fmt(obj):>10}{obj_reset} "
            f"{exp_color}{exp:>9}{exp_reset} "
            f"{r['time_ms']:>7.1f}ms"
        )

    # path details
    print()
    for r in rows:
        path_str = " -> ".join(r["path"][:10])
        if len(r["path"]) > 10:
            path_str += " ..."
        print(f"  {DIM}{r['name']:<22}{RESET} path: {path_str}")


#  Build a hand-crafted "fuel price trap" graph


def build_fuel_trap_graph():
    
    g = Graph()
    prices = {"S": 3.0, "A1": 9.0, "B1": 2.0, "G": 3.0}
    for n, p in prices.items():
        g.add_node(n, p)

    g.add_edge("S", "A1", 5.0)
    g.add_edge("A1", "G", 5.0)
    g.add_edge("S", "B1", 6.0)
    g.add_edge("B1", "G", 6.0)

    positions = {
        "S":  (0.0, 0.0),
        "A1": (5.0, 1.0),
        "B1": (5.0, -1.0),
        "G":  (10.0, 0.0),
    }

    vehicle = Vehicle(tank_capacity=10, fuel=2, consumption_per_dist=0.5)
    return g, positions, vehicle, prices




def run_scenario(label, g, positions, vehicle, start, goal, weights, fuel_prices=None):
    section(label)

    # Print graph info
    node_names = list(positions.keys())
    print(f"  Nodes : {len(node_names)}  ({', '.join(node_names[:8])}{'...' if len(node_names) > 8 else ''})")
    if fuel_prices:
        price_str = ", ".join(f"{n}=${p:.2f}" for n, p in fuel_prices.items())
        print(f"  Prices: {price_str}")
    print(f"  Vehicle: tank={vehicle.tank_capacity}  fuel={vehicle.fuel}  "
          f"consumption={vehicle.consumption_per_dist}/dist")
    print(f"  Route : {start} -> {goal}   "
          f"weights: dist={weights['distance']}, fuel={weights['fuel']}")

    algos = [
        ("Dijkstra",         DijkstraFuel()),
        ("Greedy",           GreedyCheapFuel()),
        ("A* Original",      AStarFuel()),
        ("A* Optimized",     AStarFuelOptimized()),
    ]

    rows = []
    for name, algo in algos:
        # reset vehicle fuel each run
        v = Vehicle(
            tank_capacity=vehicle.tank_capacity,
            fuel=vehicle.fuel,
            consumption_per_dist=vehicle.consumption_per_dist,
        )
        t0 = time.perf_counter()
        try:
            result = algo.solve(g, start, goal, v, weights, positions)
        except Exception as e:
            result = {
                "path": [], "total_distance": float("inf"),
                "fuel_cost": float("inf"), "objective": float("inf"),
                "expanded": 0, "notes": str(e),
            }
        elapsed = (time.perf_counter() - t0) * 1000

        rows.append({
            "name": name,
            "distance": result["total_distance"],
            "fuel_cost": result["fuel_cost"],
            "objective": result["objective"],
            "expanded": result["expanded"],
            "time_ms": elapsed,
            "path": result["path"],
        })

    print_table(rows)
    return rows



#  Summary scoring

def print_summary(all_results):
    banner("SUMMARY")

    algo_names = ["Dijkstra", "Greedy", "A* Original", "A* Optimized"]
    wins = {n: 0 for n in algo_names}
    total_expanded = {n: 0 for n in algo_names}
    total_objective = {n: 0.0 for n in algo_names}
    scenario_count = 0

    for scenario_rows in all_results:
        feasible = [r for r in scenario_rows if r["objective"] < float("inf")]
        if not feasible:
            continue
        scenario_count += 1
        best = min(r["objective"] for r in feasible)
        for r in scenario_rows:
            if r["name"] in algo_names:
                total_expanded[r["name"]] += r["expanded"]
                if r["objective"] < float("inf"):
                    total_objective[r["name"]] += r["objective"]
                if abs(r["objective"] - best) < 1e-6:
                    wins[r["name"]] += 1

    print(f"\n  Across {scenario_count} scenarios:\n")
    print(f"  {'Algorithm':<22} {'Wins':>6} {'Total Expanded':>15} {'Total Obj':>12}")
    print(f"  {'-' * 22} {'-' * 6} {'-' * 15} {'-' * 12}")
    for n in algo_names:
        w_color = GREEN if wins[n] == max(wins.values()) else ""
        e_color = GREEN if total_expanded[n] == min(total_expanded.values()) else ""
        print(
            f"  {n:<22} "
            f"{w_color}{wins[n]:>6}{RESET if w_color else ''} "
            f"{e_color}{total_expanded[n]:>15}{RESET if e_color else ''} "
            f"{total_objective[n]:>12.2f}"
        )

    # Direct comparison
    orig_exp = total_expanded.get("A* Original", 0)
    opt_exp = total_expanded.get("A* Optimized", 0)
    if orig_exp > 0:
        reduction = ((orig_exp - opt_exp) / orig_exp) * 100
        print(f"\n  {BOLD}A* Optimized vs Original:{RESET}")
        print(f"    Node expansions: {orig_exp} -> {opt_exp}  "
              f"({GREEN}{reduction:+.1f}%{RESET} reduction)")
    print()



#  Main


def main():
    banner("Fuel-Aware A* Heuristic Evaluation")
    print(f"\n  Comparing 4 algorithms across multiple scenarios.")
    print(f"  The optimized A* uses a tighter heuristic that accounts")
    print(f"  for fuel purchase costs, not just distance.\n")

    all_results = []

    #  Scenario 1: Fuel Price Trap (hand-crafted) 
    g, pos, v, prices = build_fuel_trap_graph()
    weights = {"distance": 1.0, "fuel": 1.5}
    rows = run_scenario(
        "Scenario 1 -- Fuel Price Trap  (short+expensive vs long+cheap)",
        g, pos, v, "S", "G", weights, prices,
    )
    all_results.append(rows)

    #  Scenario 2: Random graph, balanced weights 
    g2, pos2 = generate_random_graph(n=10, edge_prob=0.3, seed=42)
    v2 = Vehicle(tank_capacity=20, fuel=5, consumption_per_dist=0.08)
    prices2 = {n: g2.fuel_price(n) for n in list(pos2.keys())[:6]}
    rows = run_scenario(
        "Scenario 2 -- Random 10-node (seed=42, balanced weights)",
        g2, pos2, v2, "N0", "N9",
        {"distance": 1.0, "fuel": 1.0}, prices2,
    )
    all_results.append(rows)

    #  Scenario 3: Random graph, fuel-heavy weights 
    g3, pos3 = generate_random_graph(n=12, edge_prob=0.28, seed=99,
                                      price_low=2.0, price_high=8.0)
    v3 = Vehicle(tank_capacity=15, fuel=3, consumption_per_dist=0.10)
    prices3 = {n: g3.fuel_price(n) for n in list(pos3.keys())[:6]}
    rows = run_scenario(
        "Scenario 3 -- Random 12-node (seed=99, fuel-heavy w_fuel=2.0)",
        g3, pos3, v3, "N0", "N11",
        {"distance": 1.0, "fuel": 2.0}, prices3,
    )
    all_results.append(rows)

    # Scenario 4: Larger graph, distance-heavy 
    g4, pos4 = generate_random_graph(n=12, edge_prob=0.35, seed=7,
                                      price_low=3.5, price_high=5.5)
    v4 = Vehicle(tank_capacity=25, fuel=8, consumption_per_dist=0.06)
    prices4 = {n: g4.fuel_price(n) for n in list(pos4.keys())[:6]}
    rows = run_scenario(
        "Scenario 4 -- Random 12-node (seed=7, distance-heavy w_dist=2.0)",
        g4, pos4, v4, "N0", "N11",
        {"distance": 2.0, "fuel": 1.0}, prices4,
    )
    all_results.append(rows)

    # Scenario 5: Tight fuel budget 
    g5, pos5 = generate_random_graph(n=8, edge_prob=0.4, seed=123,
                                      price_low=4.0, price_high=7.0)
    v5 = Vehicle(tank_capacity=8, fuel=1, consumption_per_dist=0.12)
    prices5 = {n: g5.fuel_price(n) for n in list(pos5.keys())[:6]}
    rows = run_scenario(
        "Scenario 5 -- 8-node tight fuel budget (tank=8, fuel=1)",
        g5, pos5, v5, "N0", "N7",
        {"distance": 1.0, "fuel": 1.5}, prices5,
    )
    all_results.append(rows)

    #  Summary 
    print_summary(all_results)


if __name__ == "__main__":
    main()
