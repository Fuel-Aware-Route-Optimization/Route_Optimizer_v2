"use client";

import React from "react";
import { MapContainer, TileLayer, Marker, Popup, Polyline, useMap } from "react-leaflet";
import L from "leaflet";
import "leaflet/dist/leaflet.css";

if (typeof window !== "undefined") {
  delete (L.Icon.Default.prototype as any)._getIconUrl;
  L.Icon.Default.mergeOptions({
    iconRetinaUrl: "/marker-icon-2x.png",
    iconUrl: "/marker-icon.png",
    shadowUrl: "/marker-shadow.png",
  });
}

import icon from "leaflet/dist/images/marker-icon.png";
import iconShadow from "leaflet/dist/images/marker-shadow.png";
import iconRetina from "leaflet/dist/images/marker-icon-2x.png";

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
  geometry?: number[][];
};

type RouteMapProps = {
  nodes: Node[];
  edges: Edge[];
  routePath: NodeId[];
  fromNode?: NodeId;
  toNode?: NodeId;
};

// Custom icons for start/end markers
const createCustomIcon = (color: string, label: string) => {
  return L.divIcon({
    className: "custom-marker",
    html: `
      <div style="
        background-color: ${color};
        width: 32px;
        height: 32px;
        border-radius: 50%;
        border: 3px solid white;
        box-shadow: 0 2px 8px rgba(0,0,0,0.3);
        display: flex;
        align-items: center;
        justify-content: center;
        font-weight: bold;
        color: white;
        font-size: 14px;
      ">${label}</div>
    `,
    iconSize: [32, 32],
    iconAnchor: [16, 16],
    popupAnchor: [0, -16],
  });
};

const createNodeIcon = (isOnPath: boolean, label: string) => {
  const color = isOnPath ? "#0EA5E9" : "#64748B";
  return L.divIcon({
    className: "custom-marker",
    html: `
      <div style="
        background-color: white;
        width: 28px;
        height: 28px;
        border-radius: 50%;
        border: 3px solid ${color};
        box-shadow: 0 2px 6px rgba(0,0,0,0.2);
        display: flex;
        align-items: center;
        justify-content: center;
        font-weight: bold;
        color: #1e293b;
        font-size: 12px;
      ">${label}</div>
    `,
    iconSize: [28, 28],
    iconAnchor: [14, 14],
    popupAnchor: [0, -14],
  });
};

const FitBounds: React.FC<{ nodes: Node[] }> = ({ nodes }) => {
  const map = useMap();

  React.useEffect(() => {
    if (nodes.length > 0) {
      const bounds = L.latLngBounds(
        nodes.map((node) => [node.y, node.x] as [number, number])
      );
      map.fitBounds(bounds, { padding: [50, 50] });
    }
  }, [nodes, map]);

  return null;
};

export default function RouteMap({
  nodes,
  edges,
  routePath,
  fromNode,
  toNode,
}: RouteMapProps) {
  const [isMounted, setIsMounted] = React.useState(false);
  const [mapId] = React.useState(() => `map-${Math.random().toString(36).substr(2, 9)}`);

  React.useEffect(() => {
    setIsMounted(true);
    return () => setIsMounted(false);
  }, []);

  const nodesById = React.useMemo(() => {
    const lookup: Record<NodeId, Node> = {};
    nodes.forEach((n) => {
      lookup[n.id] = n;
    });
    return lookup;
  }, [nodes]);

  const pathEdges = React.useMemo(() => {
    const edgeSet = new Set<string>();
    if (routePath.length > 1) {
      for (let i = 0; i < routePath.length - 1; i++) {
        const [a, b] = [routePath[i], routePath[i + 1]].sort();
        edgeSet.add(`${a}|${b}`);
      }
    }
    return edgeSet;
  }, [routePath]);

  const pathNodes = React.useMemo(() => new Set(routePath), [routePath]);

  const center: [number, number] = React.useMemo(() => {
    if (nodes.length === 0) return [37.7749, -122.4194]; // Default to SF
    const avgY = nodes.reduce((sum, n) => sum + n.y, 0) / nodes.length;
    const avgX = nodes.reduce((sum, n) => sum + n.x, 0) / nodes.length;
    return [avgY, avgX];
  }, [nodes]);

  const edgeLines = React.useMemo(() => {
    return edges
      .map((edge) => {
        const fromNodeData = nodesById[edge.from_];
        const toNodeData = nodesById[edge.to];
        if (!fromNodeData || !toNodeData) return null;

        const [a, b] = [edge.from_, edge.to].sort();
        const isOnPath = pathEdges.has(`${a}|${b}`);

        let positions: [number, number][];
        if (edge.geometry && edge.geometry.length > 0) {
          positions = edge.geometry.map(([lon, lat]) => [lat, lon] as [number, number]);
        } else {
          positions = [
            [fromNodeData.y, fromNodeData.x] as [number, number],
            [toNodeData.y, toNodeData.x] as [number, number],
          ];
        }

        return {
          positions,
          isOnPath,
          distance: edge.distance,
        };
      })
      .filter(Boolean) as Array<{
      positions: [number, number][];
      isOnPath: boolean;
      distance: number;
    }>;
  }, [edges, nodesById, pathEdges]);

  if (!isMounted) {
    return (
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
    );
  }

  if (nodes.length === 0) {
    return (
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
        <p style={{ color: "#64748b" }}>No nodes to display on map</p>
      </div>
    );
  }

  return (
    <div style={{ width: "100%", height: "500px", borderRadius: 10, overflow: "hidden" }} id={mapId}>
      <MapContainer
        key={mapId}
        center={center}
        zoom={12}
        style={{ width: "100%", height: "100%" }}
        scrollWheelZoom={true}
        attributionControl={true}
        zoomControl={true}
      >
        <TileLayer
          attribution='&copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> contributors'
          url="https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png"
        />
        
        <FitBounds nodes={nodes} />

        {edgeLines.map((line, idx) => (
          <Polyline
            key={`edge-${idx}`}
            positions={line.positions}
            pathOptions={{
              color: line.isOnPath ? "#0EA5E9" : "#94A3B8",
              weight: line.isOnPath ? 5 : 2,
              opacity: line.isOnPath ? 0.9 : 0.5,
            }}
          >
            <Popup>
              <div style={{ textAlign: "center" }}>
                <strong>Distance:</strong> {line.distance.toFixed(2)} units
                {line.isOnPath && (
                  <div style={{ color: "#0EA5E9", marginTop: 4 }}>
                    ✓ On route
                  </div>
                )}
              </div>
            </Popup>
          </Polyline>
        ))}

        {nodes.map((node) => {
          const isStart = node.id === fromNode;
          const isEnd = node.id === toNode;
          const isOnPath = pathNodes.has(node.id);

          let icon;
          if (isStart) {
            icon = createCustomIcon("#10B981", "S");
          } else if (isEnd) {
            icon = createCustomIcon("#EF4444", "E");
          } else {
            icon = createNodeIcon(isOnPath, node.id);
          }

          return (
            <Marker
              key={node.id}
              position={[node.y, node.x]}
              icon={icon}
            >
              <Popup>
                <div style={{ textAlign: "center" }}>
                  <strong>Node {node.id}</strong>
                  <br />
                  <small>
                    Lat: {node.y.toFixed(4)}, Lng: {node.x.toFixed(4)}
                  </small>
                  {isStart && (
                    <div style={{ color: "#10B981", marginTop: 4 }}>
                      🚩 Start
                    </div>
                  )}
                  {isEnd && (
                    <div style={{ color: "#EF4444", marginTop: 4 }}>
                      🏁 End
                    </div>
                  )}
                  {isOnPath && !isStart && !isEnd && (
                    <div style={{ color: "#0EA5E9", marginTop: 4 }}>
                      ✓ On route
                    </div>
                  )}
                </div>
              </Popup>
            </Marker>
          );
        })}
      </MapContainer>
    </div>
  );
}

