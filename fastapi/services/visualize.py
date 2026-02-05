
import os
import matplotlib.pyplot as plt

def plot_graph(graph, positions, path=None, title=None, save_path=None):
    fig, ax = plt.subplots(figsize=(7,6))

    for a_name, a_node in graph.nodes.items():
        for e in a_node.edges:
            if a_name < e.to:
                ax.plot([positions[a_name][0], positions[e.to][0]],
                        [positions[a_name][1], positions[e.to][1]],
                        linewidth=1.0, alpha=0.6)
                mx = (positions[a_name][0] + positions[e.to][0]) / 2.0
                my = (positions[a_name][1] + positions[e.to][1]) / 2.0
                ax.text(mx, my, f"{e.distance:.1f}", fontsize=7, ha="center", va="center", alpha=0.7)

    xs, ys = [], []
    for name, (x,y) in positions.items():
        xs.append(x); ys.append(y)
    ax.scatter(xs, ys, s=60)

    for name, (x,y) in positions.items():
        ax.text(x, y, f" {name}\n ${graph.fuel_price(name):.2f}", fontsize=8, ha="left", va="center")

    if path and len(path) >= 2:
        px = [positions[n][0] for n in path]
        py = [positions[n][1] for n in path]
        ax.plot(px, py, linewidth=3.0)
        for i, n in enumerate(path):
            x, y = positions[n]
            ax.text(x, y, f"{i}", fontsize=8, ha="center", va="bottom")

    if title: ax.set_title(title)
    ax.set_aspect("equal", adjustable="datalim")
    ax.grid(True, linestyle="--", linewidth=0.5, alpha=0.5)
    if save_path:
        os.makedirs(os.path.dirname(save_path), exist_ok=True)
        fig.savefig(save_path, dpi=160, bbox_inches="tight")
    plt.close(fig)
    return save_path
