from typing import List, Optional, Literal
import urllib.request
import json
import urllib.parse

from fastapi import FastAPI, Query
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel

from Algorithms.dijkstra_fuel import DijkstraFuel
from Algorithms.greedy_cheap_fuel import GreedyCheapFuel
from Algorithms.astar_fuel import AStarFuel
from services.graph import generate_random_graph, Vehicle

app = FastAPI(title="Fuel-Aware Route Optimizer")

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


class EdgeOut(BaseModel):
    # use from_ in JSON as well to avoid alias issues
    from_: str
    to: str
    distance: float
    geometry: Optional[List[List[float]]] = None  # [[lon, lat], [lon, lat], ...]


class RouteOut(BaseModel):
    path: List[str]
    total_distance: float
    fuel_cost: float
    objective: float
    expanded: int
    notes: Optional[str] = None


class RouteResponse(BaseModel):
    nodes: List[NodeOut]
    edges: List[EdgeOut]
    route: RouteOut


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
    # Generate graph with real US city coordinates
    g, positions = generate_random_graph(n=12, edge_prob=0.3, seed=seed)

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
        return RouteResponse(nodes=[], edges=[], route=empty_route)

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

    # Build response data structures
    nodes_out: List[NodeOut] = []
    for node_id, (x, y) in positions.items():
        nodes_out.append(NodeOut(id=str(node_id), x=float(x), y=float(y)))

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

    return RouteResponse(nodes=nodes_out, edges=edges_out, route=route_out)
