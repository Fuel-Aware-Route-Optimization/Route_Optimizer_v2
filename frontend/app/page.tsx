// app/page.tsx
"use client";
import React from "react";
import dynamic from "next/dynamic";

// Dynamically import the map component to avoid SSR issues with Leaflet
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
};

type Edge = {
  from_: NodeId;
  to: NodeId;
  distance: number;
};

type Algorithm = "dijkstra" | "astar" | "greedy";

type Route = {
  path: NodeId[];
  total_distance: number;
  fuel_cost: number;
  objective: number;
  expanded: number;
  notes?: string;
};

type RouteResponse = {
  nodes: Node[];
  edges: Edge[];
  route: Route;
};

const API_BASE_URL =
  process.env.NEXT_PUBLIC_ROUTE_API_BASE_URL ?? "http://127.0.0.1:8000";

export default function Page() {
  const [nodes, setNodes] = React.useState<Node[]>([]);
  const [edges, setEdges] = React.useState<Edge[]>([]);
  const [route, setRoute] = React.useState<Route | null>(null);

  const [from, setFrom] = React.useState<NodeId | "">("");
  const [to, setTo] = React.useState<NodeId | "">("");

  const [algorithm, setAlgorithm] = React.useState<Algorithm>("dijkstra");
  const [seed, setSeed] = React.useState<string>("42");

  const [loading, setLoading] = React.useState(false);
  const [error, setError] = React.useState<string | null>(null);

  const availableNodeIds = React.useMemo(() => nodes.map((n) => n.id), [nodes]);

  const fetchRoute = async (useCurrentStartGoal: boolean) => {
    setLoading(true);
    setError(null);

    try {
      const params = new URLSearchParams();
      params.set("algorithm", algorithm);
      params.set("seed", seed.trim() || "42");

      if (useCurrentStartGoal && from && to) {
        params.set("start", from);
        params.set("goal", to);
      }

      const res = await fetch(`${API_BASE_URL}/route?${params.toString()}`);
      if (!res.ok) {
        throw new Error(`Request failed with status ${res.status}`);
      }

      const data: RouteResponse = await res.json();

      setNodes(data.nodes);
      setEdges(
        data.edges.map((e: any) => ({
          from_: e.from_,
          to: e.to,
          distance: e.distance,
        })),
      );
      setRoute(data.route);

      // On first load, initialize from/to from the route path if not set
      if (!useCurrentStartGoal && data.route.path.length > 0) {
        if (!from) setFrom(data.route.path[0]);
        if (!to) setTo(data.route.path[data.route.path.length - 1]);
      }
    } catch (err: any) {
      console.error(err);
      setError(err.message ?? "Failed to fetch");
      setRoute(null);
    } finally {
      setLoading(false);
    }
  };

  // Initial load: let backend choose start/goal
  React.useEffect(() => {
    fetchRoute(false);
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, []);

  const onSubmit = (e: React.FormEvent) => {
    e.preventDefault();
    fetchRoute(true);
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
          // Responsive: auto-fit columns of at least 170px, stacking on small screens
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
              Select node
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
              Select node
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
          Route
        </h2>

        {/* {!route ? ( */}
        {/*   <p style={{ color: "#64748B", fontSize: "0.95rem" }}> */}
        {/*     Fetch a route to see path details. */}
        {/*   </p> */}
        {/* ) : route.path.length ? ( */}
        {/*   <> */}
        {/*     <ol style={{ paddingLeft: "1.25rem", marginBottom: "0.75rem" }}> */}
        {/*       {route.path.map((node, idx) => ( */}
        {/*         <li key={`${node}-${idx}`} style={{ margin: "0.25rem 0" }}> */}
        {/*           {node} */}
        {/*         </li> */}
        {/*       ))} */}
        {/*     </ol> */}
        {/*     <div */}
        {/*       style={{ */}
        {/*         display: "flex", */}
        {/*         flexWrap: "wrap", */}
        {/*         gap: "0.5rem", */}
        {/*       }} */}
        {/*     > */}
        {/*       <div */}
        {/*         style={{ */}
        {/*           display: "inline-block", */}
        {/*           padding: "0.35rem 0.6rem", */}
        {/*           borderRadius: 999, */}
        {/*           background: "#F1F5F9", */}
        {/*           border: "1px solid #E2E8F0", */}
        {/*           fontWeight: 600, */}
        {/*           fontSize: "0.9rem", */}
        {/*         }} */}
        {/*       > */}
        {/*         Total distance: {route.total_distance.toFixed(2)} */}
        {/*       </div> */}
        {/*       <div */}
        {/*         style={{ */}
        {/*           display: "inline-block", */}
        {/*           padding: "0.35rem 0.6rem", */}
        {/*           borderRadius: 999, */}
        {/*           background: "#F1F5F9", */}
        {/*           border: "1px solid #E2E8F0", */}
        {/*           fontWeight: 600, */}
        {/*           fontSize: "0.9rem", */}
        {/*         }} */}
        {/*       > */}
        {/*         Fuel cost: {route.fuel_cost.toFixed(2)} */}
        {/*       </div> */}
        {/*     </div> */}
        {/*     {route.notes ? ( */}
        {/*       <p */}
        {/*         style={{ */}
        {/*           marginTop: "0.75rem", */}
        {/*           color: "#64748B", */}
        {/*           fontSize: "0.9rem", */}
        {/*         }} */}
        {/*       > */}
        {/*         <strong>Notes:</strong> {route.notes} */}
        {/*       </p> */}
        {/*     ) : null} */}
        {/*   </> */}
        {/* ) : ( */}
        {/*   <p style={{ color: "#991B1B" }}>No path found.</p> */}
        {/* )} */}
      </section>
    </main>
  );
}
