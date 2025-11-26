import timeit
from main import create_graph, dijkstra, astar

def time_counter(workspace, file, start, end):

    g = create_graph(workspace, file)

    if len(g.nodes) == 0:
        raise ValueError("No nodes found")

    print(f"Graph: v={len(g.nodes)}, e={len(g.edges)}, start node={start}, end node={end}")

    time_dijkstra = timeit.timeit(lambda:dijkstra(g, start, end), number=1)
    astar_time = timeit.timeit(lambda:astar(g, start, end), number=1)

    _, _, _, d_nodes, d_edges = dijkstra(g, start, end)
    _, _, _, a_nodes, a_edges = astar(g, start, end)

    print("✧RESULTS✧")
    print(f"Dijkstra: time={time_dijkstra}, v={d_nodes}, e={d_edges}")
    print(f"Astar: time={astar_time}, v={a_nodes}, e={a_edges}")

if __name__ == "__main__":
    ws = r"C:\Users\Weronika\Documents\Vsem\drogi\PL_PZGiK_994_BDOT10k_0463__OT_SKJZ_L.shp"
    f = "PL_PZGiK_994_BDOT10k_0463__OT_SKJZ_L.shp"

    time_counter(ws, f, 1000, 6767)
    time_counter(ws, f, 2222, 4444)






