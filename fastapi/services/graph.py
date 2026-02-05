
from __future__ import annotations
from dataclasses import dataclass, field
from typing import Dict, List, Tuple, Optional
import math, random

@dataclass
class Edge:
    to: str
    distance: float

@dataclass
class Node:
    name: str
    fuel_price: float
    edges: List[Edge] = field(default_factory=list)

class Graph:
    def __init__(self):
        self.nodes: Dict[str, Node] = {}

    def add_node(self, name: str, fuel_price: float):
        if name not in self.nodes:
            self.nodes[name] = Node(name, fuel_price)

    def add_edge(self, a: str, b: str, distance: float):
        if a not in self.nodes or b not in self.nodes:
            raise ValueError("Both nodes must exist before adding an edge.")
        self.nodes[a].edges.append(Edge(b, distance))
        self.nodes[b].edges.append(Edge(a, distance))

    def neighbors(self, name: str) -> List[Edge]:
        return self.nodes[name].edges

    def fuel_price(self, name: str) -> float:
        return self.nodes[name].fuel_price

@dataclass
class Vehicle:
    tank_capacity: float
    fuel: float
    consumption_per_dist: float
    max_refuels: Optional[int] = None

def generate_random_graph(n: int = 12,
                          edge_prob: float = 0.28,
                          seed: int = 42,
                          price_low: float = 3.0,
                          price_high: float = 6.0,
                          pos_spread: float = 50.0):
    rng = random.Random(seed)
    g = Graph()
    positions = {}
    
    # Real US cities along I-10 and I-20 corridors from Arizona to Texas
    highway_cities = [
        ("Phoenix, AZ", -112.074, 33.448),
        ("Tucson, AZ", -110.974, 32.222),
        ("Las Cruces, NM", -106.779, 32.312),
        ("El Paso, TX", -106.487, 31.762),
        ("Midland, TX", -102.078, 31.997),
        ("San Angelo, TX", -100.437, 31.464),
        ("Abilene, TX", -99.733, 32.449),
        ("Fort Worth, TX", -97.331, 32.756),
        ("Dallas, TX", -96.797, 32.776),
        ("Tyler, TX", -95.301, 32.351),
        ("Houston, TX", -95.369, 29.760),
        ("San Antonio, TX", -98.493, 29.424),
    ]
    
    selected_cities = highway_cities[:n]
    
    for i, (city, lon, lat) in enumerate(selected_cities):
        name = f"N{i}"
        price = rng.uniform(price_low, price_high)
        g.add_node(name, round(price, 2))
        jitter_lon = rng.uniform(-0.3, 0.3)
        jitter_lat = rng.uniform(-0.3, 0.3)
        positions[name] = (lon + jitter_lon, lat + jitter_lat)

    names = list(positions.keys())
    for i in range(n):
        for j in range(i+1, n):
            ax, ay = positions[names[i]]
            bx, by = positions[names[j]]
            distance_miles = math.hypot((ax-bx) * 54, (ay-by) * 69)
            
            is_adjacent = (j - i) == 1
            is_nearby = distance_miles < 300
            
            if is_adjacent or (is_nearby and rng.random() < edge_prob):
                d = distance_miles * rng.uniform(0.95, 1.05)
                g.add_edge(names[i], names[j], round(d, 2))

    # Ensure all cities are connected
    for i in range(n-1):
        a = f"N{i}"; b = f"N{i+1}"
        if all(e.to != b for e in g.nodes[a].edges) and all(e.to != a for e in g.nodes[b].edges):
            ax, ay = positions[a]; bx, by = positions[b]
            d = math.hypot((ax-bx) * 54, (ay-by) * 69)
            g.add_edge(a, b, round(d, 2))

    return g, positions
