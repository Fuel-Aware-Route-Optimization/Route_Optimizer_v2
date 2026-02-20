"use client";
import React from "react";
import dynamic from "next/dynamic";

const RouteMap = dynamic(() => import("./components/RouteMap"), {
  ssr: false,
  loading: () => (
    <div
      style={{
        width: "100%",
        height: "500px",
        display: "flex",
        alignItems: "center",
        justifyContent: "center",
        background: "#f8fafc",
        border: "1px solid #e2e8f0",
        borderRadius: 10,
      }}
    >
      <p style={{ color: "#64748b" }}>Loading map...</p>
    </div>
  ),
});

type NodeId = string;

type Node = {
  id: NodeId;
  x: number;
  y: number;
  fuel_price?: number;
};

type Edge = {
  from_: NodeId;
  to: NodeId;
  distance: number;
  geometry?: number[][];
};

type Algorithm = "dijkstra" | "astar" | "greedy";

type RouteSummary = {
  distance_km: number;
  fuel_cost: number;
};

type Route = {
  path: NodeId[];
  total_distance: number;
  fuel_cost: number;
  objective: number;
  expanded: number;
  notes?: string;
  summary?: RouteSummary; 
};

type RouteComparison = {
  baseline_fuel_cost: number;
  optimized_fuel_cost: number;
  savings_amount: number;
  savings_percent: number;
};

type RouteResponse = {
  api_version?: string;
  nodes: Node[];
  edges: Edge[];
  route: Route;
  baseline_path?: NodeId[];
  comparison?: RouteComparison;
};

const API_BASE_URL =
  process.env.NEXT_PUBLIC_ROUTE_API_BASE_URL ?? "http://127.0.0.1:8000";


export default function Page() {
  const [nodes, setNodes] = React.useState<Node[]>([]);
  const [edges, setEdges] = React.useState<Edge[]>([]);
  const [comparison, setComparison] = React.useState<RouteComparison | null>(null);
  const [route, setRoute] = React.useState<Route | null>(null);
  const [baselinePath, setBaselinePath] = React.useState<NodeId[]>([]);

  const [from, setFrom] = React.useState<NodeId | "">("");
  const [to, setTo] = React.useState<NodeId | "">("");

  const [algorithm, setAlgorithm] = React.useState<Algorithm>("dijkstra");
  const [seed, setSeed] = React.useState<string>("42");

  const [loading, setLoading] = React.useState(false);
  const [error, setError] = React.useState<string | null>(null);

  const availableNodeIds = React.useMemo(() => nodes.map((n) => n.id), [nodes]);
  const summaryPath = React.useMemo(() => {
    if (!route?.path?.length) return [];
    const cleaned: NodeId[] = [];
    for (const node of route.path) {
      if (cleaned.length === 0 || cleaned[cleaned.length - 1] !== node) {
        cleaned.push(node);
      }
    }
    return cleaned;
  }, [route]);

  const fetchRoute = React.useCallback(async (
    useCurrentStartGoal: boolean,
    selectedFrom: NodeId | "",
    selectedTo: NodeId | "",
  ) => {
    setLoading(true);
    setError(null);

    try {
      const params = new URLSearchParams();
      params.set("algorithm", algorithm);
      params.set("seed", seed.trim() || "42");

      if (useCurrentStartGoal && selectedFrom && selectedTo) {
        params.set("start", selectedFrom);
        params.set("goal", selectedTo);
      }

      const res = await fetch(`${API_BASE_URL}/route?${params.toString()}`);
      if (!res.ok) {
        throw new Error(`Request failed with status ${res.status}`);
      }

      const data: RouteResponse = await res.json();


      if (data.api_version && data.api_version !== "v1") {
  
        console.info(`Route API returned version: ${data.api_version}`);
      }


      const nodesWithFuel = data.nodes.map((node) => ({ ...node }));

      const apiRoute = data.route ?? null;
      if (apiRoute) {
    if (!apiRoute.summary) {
      apiRoute.summary = {
      distance_km: apiRoute.total_distance ?? 0,
      fuel_cost: apiRoute.fuel_cost ?? 0,
      };
    }
  }

    setNodes(nodesWithFuel);
    setEdges(
      data.edges.map((e) => ({
      from_: e.from_,
      to: e.to,
      distance: e.distance,
      geometry: e.geometry,
      })),
    );

    setRoute(apiRoute);
    setBaselinePath(data.baseline_path ?? []);

    setComparison(data.comparison ?? null);


      if (!useCurrentStartGoal && data.route.path.length > 0) {
        if (!selectedFrom) setFrom(data.route.path[0]);
        if (!selectedTo) setTo(data.route.path[data.route.path.length - 1]);
      }
    } catch (err: unknown) {
      console.error(err);
      setError(err instanceof Error ? err.message : "Failed to fetch");
      setRoute(null);
      setBaselinePath([]);
    } finally {
      setLoading(false);
    }
  }, [algorithm, seed]);

  const didInitialFetch = React.useRef(false);
  React.useEffect(() => {
    if (didInitialFetch.current) return;
    didInitialFetch.current = true;
    fetchRoute(false, "", "");
  }, [fetchRoute]);

  const onSubmit = (e: React.FormEvent) => {
    e.preventDefault();
    fetchRoute(true, from, to);
  };

  return (
    <main
      style={{
        fontFamily: "ui-sans-serif, system-ui, -apple-system, Segoe UI, Roboto",
        padding: "1.5rem",
        maxWidth: 900,
        width: "100%",
        margin: "0 auto",
        lineHeight: 1.6,
        boxSizing: "border-box",
      }}
    >
      <h1
        style={{
          fontSize: "1.5rem",
          fontWeight: 700,
          marginBottom: "0.5rem",
        }}
      >
        Fuel-Aware Route Optimizer with Interactive Map
      </h1>
      <p
        style={{
          fontSize: "0.95rem",
          color: "#64748B",
          marginBottom: "1rem",
        }}
      >
        Visualize optimal freight routes with fuel station data on an
        interactive map
      </p>

      {/* Controls */}
      <form
        onSubmit={onSubmit}
        style={{
          display: "grid",
          gridTemplateColumns: "repeat(auto-fit, minmax(170px, 1fr))",
          gap: "0.75rem",
          alignItems: "end",
          marginBottom: "1rem",
        }}
      >
        <label style={{ display: "grid", gap: "0.25rem" }}>
          <span style={{ fontSize: "0.875rem", color: "#475569" }}>From</span>
          <select
            value={from}
            onChange={(e) => setFrom(e.target.value as NodeId)}
            disabled={!availableNodeIds.length}
            style={{
              border: "1px solid #CBD5E1",
              borderRadius: 6,
              padding: "0.5rem 0.625rem",
              background: "white",
              width: "100%",
            }}
          >
            <option value="" disabled>
              Select city
            </option>
            {availableNodeIds.map((id) => (
              <option key={id} value={id}>
                {id}
              </option>
            ))}
          </select>
        </label>

        <label style={{ display: "grid", gap: "0.25rem" }}>
          <span style={{ fontSize: "0.875rem", color: "#475569" }}>To</span>
          <select
            value={to}
            onChange={(e) => setTo(e.target.value as NodeId)}
            disabled={!availableNodeIds.length}
            style={{
              border: "1px solid #CBD5E1",
              borderRadius: 6,
              padding: "0.5rem 0.625rem",
              background: "white",
              width: "100%",
            }}
          >
            <option value="" disabled>
              Select city
            </option>
            {availableNodeIds.map((id) => (
              <option key={id} value={id}>
                {id}
              </option>
            ))}
          </select>
        </label>

        <label style={{ display: "grid", gap: "0.25rem" }}>
          <span style={{ fontSize: "0.875rem", color: "#475569" }}>
            Algorithm
          </span>
          <select
            value={algorithm}
            onChange={(e) => setAlgorithm(e.target.value as Algorithm)}
            style={{
              border: "1px solid #CBD5E1",
              borderRadius: 6,
              padding: "0.5rem 0.625rem",
              background: "white",
              width: "100%",
            }}
          >
            <option value="dijkstra">Dijkstra (fuel-aware)</option>
            <option value="astar">A*</option>
            <option value="greedy">Greedy cheap fuel</option>
          </select>
        </label>

        <div
          style={{
            display: "flex",
            alignItems: "flex-end",
          }}
        >
          <button
            type="submit"
            disabled={loading || !from || !to}
            style={{
              border: "1px solid #0EA5E9",
              background: loading ? "#BAE6FD" : "#0EA5E9",
              color: "white",
              borderRadius: 8,
              padding: "0.6rem 0.9rem",
              cursor: loading ? "default" : "pointer",
              fontWeight: 600,
              whiteSpace: "nowrap",
              height: 40,
              width: "100%",
            }}
            aria-label="Compute path"
            title="Compute path"
          >
            {loading ? "Computing..." : "Show path"}
          </button>
        </div>
      </form>

      {/* Seed input */}
      <div
        style={{
          display: "flex",
          flexWrap: "wrap",
          alignItems: "center",
          gap: "0.5rem",
          marginBottom: "1rem",
        }}
      >
        <label
          style={{
            display: "inline-flex",
            gap: "0.5rem",
            alignItems: "center",
          }}
        >
          <span style={{ fontSize: "0.875rem", color: "#475569" }}>
            Graph seed
          </span>
          <input
            type="number"
            value={seed}
            onChange={(e) => setSeed(e.target.value)}
            style={{
              border: "1px solid #CBD5E1",
              borderRadius: 6,
              padding: "0.3rem 0.5rem",
              width: 90,
            }}
          />
        </label>
        <span
          style={{
            fontSize: "0.8rem",
            color: "#94A3B8",
          }}
        >
          (Changes affect the next request)
        </span>
      </div>

      {/* Error state */}
      {error && (
        <div
          style={{
            marginBottom: "1rem",
            padding: "0.75rem 1rem",
            borderRadius: 8,
            border: "1px solid #FCA5A5",
            background: "#FEF2F2",
            color: "#B91C1C",
            fontSize: "0.9rem",
          }}
        >
          {error}
        </div>
      )}

      {/* Map Visualization */}
      <section
        style={{
          border: "1px solid #E2E8F0",
          borderRadius: 10,
          padding: "0.75rem",
          marginBottom: "1rem",
          background: "#FFFFFF",
        }}
      >
        <RouteMap
          nodes={nodes}
          edges={edges}
          routePath={route?.path ?? []}
          baselineRoutePath={baselinePath}
          fromNode={from || undefined}
          toNode={to || undefined}
        />
      </section>

      {/* Route details */}
      <section
        style={{
          border: "1px solid #E2E8F0",
          borderRadius: 10,
          padding: "1rem",
          background: "#FFFFFF",
        }}
      >
        <h2
          style={{
            fontSize: "1.125rem",
            fontWeight: 600,
            marginBottom: "0.5rem",
          }}
        >
          Route Summary
        </h2>

      {!route ? (
        <p style={{ color: "#64748B", fontSize: "0.95rem" }}>
          Fetch a route to see the summary.
        </p>
      ) : route.path && route.path.length ? (
        <>
          <div
            style={{
              display: "grid",
              gridTemplateColumns: "repeat(auto-fit, minmax(180px, 1fr))",
              gap: "0.75rem",
              marginBottom: "0.75rem",
            }}
          >
            <div
              style={{
                borderRadius: 10,
                border: "1px solid #E2E8F0",
                padding: "0.65rem 0.75rem",
                background: "#F8FAFC",
                fontWeight: 600,
                color: "#0F172A",
              }}
            >
              Distance
              <div style={{ fontSize: "1.1rem", marginTop: "0.15rem" }}>
                {Number(route.summary?.distance_km ?? route.total_distance).toFixed(2)} km
              </div>
            </div>
            <div
              style={{
                borderRadius: 10,
                border: "1px solid #E2E8F0",
                padding: "0.65rem 0.75rem",
                background: "#F8FAFC",
                fontWeight: 600,
                color: "#0F172A",
              }}
            >
              Fuel cost
              <div style={{ fontSize: "1.1rem", marginTop: "0.15rem" }}>
                ${Number(route.summary?.fuel_cost ?? route.fuel_cost).toFixed(2)}
              </div>
            </div>
            <div
              style={{
                borderRadius: 10,
                border: "1px solid #E2E8F0",
                padding: "0.45rem 0.15rem",
                background: "#F8FAFC",
                fontWeight: 600,
                color: "#0F172A",
              }}
            >
              {comparison ? (
                
                <div style={{ marginTop: "0.15rem", color: "#0F172A" }}>
                  <div style={{ fontSize: "0.95rem", marginBottom: "0.25rem", fontWeight: 600 }}>
                  Cost Comparison
                  </div>
                  <div style={{ display: "flex", gap: "0.40rem", alignItems: "center" }}>
                    <div style={{ padding: "0.25rem 0.5rem", borderRadius: 8, background: "#F8FAFC", border: "1px solid #E2E8F0" }}>
                      Baseline: ${comparison.baseline_fuel_cost.toFixed(2)}
                    </div>
                    <div style={{ padding: "0.25rem 0.5rem", borderRadius: 8, background: "#ECFEFF", border: "1px solid #BDEBF8" }}>
                      Optimized: ${comparison.optimized_fuel_cost.toFixed(2)}
                    </div>
                    
                  </div>
                </div>
              ) : null}
              Legs
              <div style={{ fontSize: "1.1rem", marginTop: "0.15rem" }}>
                {Math.max(summaryPath.length - 1, 0)}
              </div>
            </div>
            <div
              style={{
                borderRadius: 10,
                border: "1px solid #E2E8F0",
                padding: "0.65rem 0.75rem",
                background: "#F8FAFC",
                fontWeight: 600,
                color: "#0F172A",
              }}
            >
              Expanded
              <div style={{ fontSize: "1.1rem", marginTop: "0.15rem" }}>
                {route.expanded}
              </div>
            </div>
          </div>

          <div
            style={{
              display: "flex",
              flexWrap: "wrap",
              gap: "0.4rem",
              alignItems: "center",
            }}
          >
            {summaryPath.map((node, idx) => (
              <React.Fragment key={`${node}-${idx}`}>
                <span
                  style={{
                    padding: "0.3rem 0.6rem",
                    borderRadius: 999,
                    background: idx === 0 ? "#DCFCE7" : idx === summaryPath.length - 1 ? "#FEE2E2" : "#E2E8F0",
                    color: "#0F172A",
                    fontWeight: 600,
                    fontSize: "0.9rem",
                  }}
                >
                  {node}
                </span>
                {idx < summaryPath.length - 1 && (
                  <span style={{ color: "#64748B", fontWeight: 600 }}>-&gt;</span>
                )}
              </React.Fragment>
            ))}
          </div>

        </>
      ) : (
        <p style={{ color: "#991B1B" }}>No path found.</p>
      )}


      </section>
    </main>
  );
}
