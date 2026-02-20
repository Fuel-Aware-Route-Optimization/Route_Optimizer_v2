from typing import Dict, List, Optional, Literal
import urllib.request
import json
import urllib.parse
from pathlib import Path

from fastapi import FastAPI, Query
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel, Field

from Algorithms.dijkstra_fuel import DijkstraFuel
from Algorithms.greedy_cheap_fuel import GreedyCheapFuel
from Algorithms.astar_fuel import AStarFuel
from services.graph import generate_random_graph, Vehicle

app = FastAPI(title="Fuel-Aware Route Optimizer")

FUEL_PRICE_SOURCE: Literal["json", "random"] = "json"   # Fuel price source toggle
# set to "json" to load prices from fuel_prices.json
# set to "random" to generate prices randomly using the graph seed

_FUEL_PRICES_PATH = Path(__file__).parent / "data" / "fuel_prices.json"

DEFAULT_FUEL_PRICE = 3.50 


def _load_fuel_prices_from_json() -> Dict[str, float]:
    with open(_FUEL_PRICES_PATH, "r", encoding="utf-8") as f:
        data = json.load(f)
    return {city: float(price) for city, price in data["prices"].items()}

def compute_fuel_cost_from_distance_km(distance_km: float, vehicle) -> float:
    try:
        fuel_used = distance_km * float(getattr(vehicle, "consumption_per_dist", 0.0))
        return float(fuel_used * DEFAULT_FUEL_PRICE)
    except Exception:
        return 0.0

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_methods=["*"],
    allow_headers=["*"],
)


# ---------- Pydantic models ----------

class NodeOut(BaseModel):
    id: str
    x: float
    y: float
    fuel_price: float


class EdgeOut(BaseModel):
    from_: str
    to: str
    distance: float
    geometry: Optional[List[List[float]]] = None  


class RouteSummary(BaseModel):
    distance_km: float
    fuel_cost: float


class RouteOut(BaseModel):
    path: List[str]
    total_distance: float
    fuel_cost: float
    objective: float
    expanded: int
    notes: Optional[str] = None
    summary: Optional[RouteSummary] = None   

class RouteComparison(BaseModel):
    baseline_fuel_cost: float
    optimized_fuel_cost: float
    savings_amount: float
    savings_percent: float

class RouteResponse(BaseModel):
    api_version: str = "v1"
    nodes: List[NodeOut]
    edges: List[EdgeOut]
    route: RouteOut
    baseline_path: List[str] = Field(default_factory=list)
    comparison: Optional[RouteComparison] = None




def get_road_route(
    lon1: float, lat1: float, lon2: float, lat2: float
) -> Optional[List[List[float]]]:
    # Call OSRM API to get actual highway routing between two points
    try:
        url = f"http://router.project-osrm.org/route/v1/driving/{lon1},{lat1};{lon2},{lat2}?overview=full&geometries=geojson"
        with urllib.request.urlopen(url, timeout=5) as response:
            data = json.loads(response.read().decode())
            if data.get("code") == "Ok" and data.get("routes"):
                return data["routes"][0]["geometry"]["coordinates"]
    except Exception as e:
        print(f"OSRM routing failed: {e}")
    return None


@app.get("/route", response_model=RouteResponse)
def get_route(
    algorithm: Literal["dijkstra", "astar", "greedy"] = "dijkstra",
    seed: int = 42,
    start: Optional[str] = Query(None),
    goal: Optional[str] = Query(None),
):
    # Generate graph: use JSON prices or random prices depending on FUEL_PRICE_SOURCE seting in Line 23
    price_overrides = _load_fuel_prices_from_json() if FUEL_PRICE_SOURCE == "json" else None
    g, positions = generate_random_graph(n=12, edge_prob=0.3, seed=seed, price_overrides=price_overrides)

    vehicle = Vehicle(tank_capacity=20, fuel=5, consumption_per_dist=0.08)
    weights = {"distance": 1.0, "fuel": 1.0}

    # Algorithm selection
    if algorithm == "astar":
        algo = AStarFuel()  # TODO: Wire up A* implementation
    elif algorithm == "greedy":
        algo = GreedyCheapFuel()  # TODO: Wire up Greedy implementation
    else:
        algo = DijkstraFuel()

    # Extract node IDs
    node_ids = list(positions.keys())
    if not node_ids:
        empty_route = RouteOut(
            path=[],
            total_distance=0.0,
            fuel_cost=0.0,
            objective=0.0,
            expanded=0,
            notes="Empty graph.",
        )
        return RouteResponse(nodes=[], edges=[], route=empty_route, baseline_path=[])

    if start is None:
        start = str(node_ids[0])
    if goal is None:
        goal = str(node_ids[-1])

    # Run routing algorithm
    result = algo.solve(g, start, goal, vehicle, weights, positions)

    path = [str(n) for n in result.get("path", [])]
    total_distance = float(result.get("total_distance", 0.0) or 0.0)
    fuel_cost = float(result.get("fuel_cost", 0.0) or 0.0)
    objective = float(result.get("objective", 0.0) or 0.0)
    expanded = int(result.get("expanded", 0) or 0)
    notes = result.get("notes")

    route_out = RouteOut(
        path=path,
        total_distance=total_distance,
        fuel_cost=fuel_cost,
        objective=objective,
        expanded=expanded,
        notes=notes,
    )

    baseline_path: List[str] = []
    try:
        baseline_weights = {"distance": 1.0, "fuel": 0.0}
        baseline_algo = DijkstraFuel()
        baseline_result = baseline_algo.solve(g, start, goal, vehicle, baseline_weights, positions)
        baseline_path = [str(n) for n in baseline_result.get("path", [])]

        baseline_total_distance = float(baseline_result.get("total_distance", 0.0) or 0.0)
        baseline_fuel_cost = float(baseline_result.get("fuel_cost", 0.0) or 0.0)

        if baseline_fuel_cost <= 0.0:
            baseline_fuel_cost = compute_fuel_cost_from_distance_km(baseline_total_distance, vehicle)
    except Exception as e:
        print(f"Baseline route calculation failed: {e}")
        baseline_path = []
        baseline_total_distance = 0.0
        baseline_fuel_cost = 0.0

    optimized_fuel_cost = fuel_cost if fuel_cost > 0 else compute_fuel_cost_from_distance_km(total_distance, vehicle)

    savings_amount = baseline_fuel_cost - optimized_fuel_cost
    savings_percent = (savings_amount / baseline_fuel_cost * 100.0) if baseline_fuel_cost > 0 else 0.0

    comparison = RouteComparison(
        baseline_fuel_cost=round(baseline_fuel_cost, 2),
        optimized_fuel_cost=round(optimized_fuel_cost, 2),
        savings_amount=round(savings_amount, 2),
        savings_percent=round(savings_percent, 2),
    )


    route_out.summary = RouteSummary(distance_km=total_distance, fuel_cost=fuel_cost)

    
    nodes_out: List[NodeOut] = []
    for node_id, (x, y) in positions.items():
        nodes_out.append(NodeOut(
            id=str(node_id),
            x=float(x),
            y=float(y),
            fuel_price=float(g.fuel_price(node_id)),
        ))

    edges_out: List[EdgeOut] = []
    seen = set()
    for u in positions.keys():
        for e in g.neighbors(u):
            v = e.to
            key = tuple(sorted((str(u), str(v))))
            if key in seen:
                continue
            seen.add(key)

            dist_val = float(getattr(e, "distance", 0.0) or 0.0)

            # Get realistic road geometry from OSRM
            u_lon, u_lat = positions[u]
            v_lon, v_lat = positions[v]
            geometry = get_road_route(u_lon, u_lat, v_lon, v_lat)

            edges_out.append(
                EdgeOut(from_=str(u), to=str(v), distance=dist_val, geometry=geometry)
            )

    return RouteResponse(
        api_version="v1",
        nodes=nodes_out,
        edges=edges_out,
        route=route_out,
        baseline_path=baseline_path,
        comparison=comparison,
    )
