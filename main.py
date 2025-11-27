import arcpy
import math
import os
from arcpy import SpatialReference
import heapq

#####słownik klasa drogi#####
Vs = {
    "autostrada": 140,
    "droga ekspresowa": 130,
    "droga główna ruchu przyspieszonego": 70,
    "droga główna": 60,
    "droga zbiorcza": 50,
    "droga lokalna": 40,
    "droga wewnętrzna": 30,
    "inna": 40
}

#=================================================#
class Node:
    def __init__(self, node_id, x, y):
        self.id = node_id  
        self.x = x  
        self.y = y  
        self.edges = []  

    def __repr__(self):
        eids = ",".join([str(e.id) for e, v in self.edges])
        return f"N({self.id},({self.x},{self.y}),[{eids}])"


class Edge:
    def __init__(self, edge_id, cost, start, end, road_class):
        self.id = edge_id  
        self.cost = cost  
        self.start = start  
        self.end = end  
        self.road_class = road_class

    def __repr__(self):
        sid = self.start.id if self.start is not None else "None"
        eid = self.end.id if self.end is not None else "None"
        return f"E({self.id},{sid},{eid})"


class Graph:
    def __init__(self):
        self.edges = {}  
        self.nodes = {}  

    def __repr__(self):
        ns = ",".join([str(n) for n in self.nodes.values()])  
        es = ",".join([str(e) for e in self.edges.values()])  
        return f"Graph:\n  Nodes: [{ns}]\n  Edges: [{es}]"


class GraphCreator:
    def __init__(self):
        self.graph = Graph()  
        self.new_id = 0  
        self.index = {}  

    def getNewId(self):
        self.new_id = self.new_id + 1
        return self.new_id

    def newNode(self, p):
        if p not in self.index:
            n = Node(self.getNewId(), p[0], p[1]) 
            self.graph.nodes[n.id] = n  
            self.index[p] = n  
        return self.index[p]

    def newEdge(self, id, length, p1, p2, road_class=None, direction=0):
        n1 = self.newNode(p1)
        n2 = self.newNode(p2)
        e = Edge(id, length, n1, n2, road_class)
        self.graph.edges[id] = e
#=================================================#
#     ©contribution: dr inż. Jacek Marciniak      #
#=================================================#

        # tylko jeśli droga przejezdna w danym kierunku
        if direction in (0, 1):
            n1.edges.append((e, n2))
        if direction in (0, 2):
            n2.edges.append((e, n1))

    def find_closest_nodes(self, point1, point2):
        def transform_point(point):
            wgs84 = SpatialReference(4326)
            target_sr = SpatialReference(2180)
            
            point_geom = arcpy.PointGeometry(
                arcpy.Point(point[0], point[1]), 
                wgs84
            )
            
            projected_point = point_geom.projectAs(target_sr)
            return (projected_point.firstPoint.X, projected_point.firstPoint.Y)
            
        def distance(p1, p2):
            return ((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)**0.5
        
        transformed_p1 = transform_point(point1)
        transformed_p2 = transform_point(point2)
        
        closest_to_p1 = None
        closest_to_p2 = None
        min_dist1 = float('inf')
        min_dist2 = float('inf')
        
        for node_id, node in self.graph.nodes.items():
            node_point = (node.x, node.y)
            
            dist1 = distance(transformed_p1, node_point)
            if dist1 < min_dist1:
                min_dist1 = dist1
                closest_to_p1 = node_id
                
            dist2 = distance(transformed_p2, node_point)
            if dist2 < min_dist2:
                min_dist2 = dist2
                closest_to_p2 = node_id
        
        print(f"\nFound closest nodes:")
        if closest_to_p1:
            node1 = self.graph.nodes[closest_to_p1]
            print(f"For point 1: Node {closest_to_p1} at ({node1.x}, {node1.y})")
        if closest_to_p2:
            node2 = self.graph.nodes[closest_to_p2]
            print(f"For point 2: Node {closest_to_p2} at ({node2.x}, {node2.y})")
        
        return closest_to_p1, closest_to_p2

def dijkstra(graph, start_id, end_id):

    Q = []
    d = {}
    p = {}
    pe = {}

    visited_nodes = 0
    visited_edges = 0

    for node_id in graph.nodes:
        d[node_id] = math.inf
        p[node_id] = None
        pe[node_id] = None

    d[start_id] = 0
    heapq.heappush(Q, (0, start_id))

    while Q:
        current_dist, v = heapq.heappop(Q)

        if current_dist > d[v]:
            continue

        visited_nodes += 1

        if v == end_id:
            break

        v_node = graph.nodes[v]

        for edge, u_node in v_node.edges:
            visited_edges += 1
            u = u_node.id
            new_dist = d[v] + edge.cost

            if new_dist < d[u]:
                d[u] = new_dist
                p[u] = v
                pe[u] = edge
                heapq.heappush(Q, (new_dist, u))

    path_nodes = []
    path_edges = []

    if d[end_id] < math.inf:
        node = end_id
        while node is not None:
            path_nodes.insert(0, node)
            edge = pe.get(node)
            if edge is not None:
                path_edges.insert(0, edge.id)
            node = p.get(node)
    else:
        print(f"Brak połączenia między wierzchołkami {start_id}, {end_id}.")
        return None, None, math.inf, 0, None

    return path_nodes, path_edges, d[end_id], visited_nodes, visited_edges


def astar(graph, start_id, end_id):

    VMAX = max(Vs.values()) / 3.6
    def heuristic(n1, n2):
        dx = n1.x - n2.x
        dy = n1.y - n2.y
        distance = math.sqrt(dx * dx + dy * dy)

        time = distance / VMAX

        return time

    Q = []
    g_cost = {}
    f_cost = {}
    p = {}
    pe = {}
    visited_nodes = 0
    visited_edges = 0
    end_node = graph.nodes[end_id]

    for node_id in graph.nodes:
        g_cost[node_id] = math.inf
        f_cost[node_id] = math.inf
        p[node_id] = None
        pe[node_id] = None

    start_node = graph.nodes[start_id]
    g_cost[start_id] = 0

    f_cost[start_id] = heuristic(start_node, end_node)
    heapq.heappush(Q, (f_cost[start_id], start_id))

    while Q:
        current_cost, v = heapq.heappop(Q)

        if current_cost > g_cost.get(v, math.inf) + heuristic(graph.nodes[v], end_node):
            continue

        visited_nodes += 1

        if v == end_id:
            break

        v_node = graph.nodes[v]

        for edge, u_node in v_node.edges:
            visited_edges += 1
            u = u_node.id

            tentative_g = g_cost[v] + edge.cost
            if tentative_g < g_cost.get(u, math.inf):
                g_cost[u] = tentative_g
                h_cost = heuristic(u_node, end_node)
                f_cost[u] = tentative_g + h_cost

                p[u] = v
                pe[u] = edge

                heapq.heappush(Q, (f_cost[u], u))

    path_nodes = []
    path_edges = []

    if g_cost[end_id] < math.inf:
        node = end_id
        while node is not None:
            path_nodes.insert(0, node)
            edge = pe[node]
            if edge is not None:
                path_edges.insert(0, edge.id)
            node = p[node]
    else:
        print(f"Brak połączenia między wierzchołkami {start_id}, {end_id}.")
        return None, None, None, None, None

    return path_nodes, path_edges, g_cost[end_id], visited_nodes, visited_edges

def create_graph(workspace, layer):
    gc = GraphCreator()
    arcpy.env.workspace = workspace
    
    fields = ['FID', 'SHAPE@', 'KLASA_DROG', 'DIRECTION']
    with arcpy.da.SearchCursor(layer, fields) as cursor:
        for row in cursor:
            fid = row[0]
            shape = row[1]
            road_class = row[2].lower()
            direction = int(row[3]) if row[3] is not None else 0
            length = round(shape.length, 2)
            v = Vs.get(str(road_class).strip(), 40)
            time = ((length / 1000.0) / v) * 3600.0 # sekundy
            p1 = (round(shape.firstPoint.X, 2), round(shape.firstPoint.Y, 2))
            p2 = (round(shape.lastPoint.X, 2), round(shape.lastPoint.Y, 2))
            gc.newEdge(fid, time, p1, p2, road_class=road_class, direction=direction)
    return gc.graph

import json

def save_graph(graph, path="graph_cache.json"):
    data = {
        "nodes": {nid: {"x": n.x, "y": n.y} for nid, n in graph.nodes.items()},
        "edges": [
            {
                "id": e.id,
                "cost": e.cost,
                "start": e.start.id,
                "end": e.end.id,
                "road_class": e.road_class
            }
            for e in graph.edges.values()
        ]
    }
    with open(path, "w", encoding="utf-8") as f:
        json.dump(data, f)
    print(f"Graf zapisany do pliku: {path}")


def load_graph(path="graph_cache.json"):

    with open(path, "r", encoding="utf-8") as f:
        data = json.load(f)

    g = Graph()
    
    for nid, ndata in data["nodes"].items():
        n = Node(int(nid), ndata["x"], ndata["y"])
        g.nodes[n.id] = n

    
    for edata in data["edges"]:
        start = g.nodes[edata["start"]]
        end = g.nodes[edata["end"]]
        e = Edge(edata["id"], edata["cost"], start, end, edata["road_class"])
        g.edges[e.id] = e
        start.edges.append((e, end))
        end.edges.append((e, start))

    print(f"Wczytano graf z pliku: {path}")
    return g


if __name__ == "__main__":
    workspace = r'C:\uni\5sem\pagdane\drogi\PL_PZGiK_994_BDOT10k_0463__OT_SKJZ_L.shp'
    layer = 'PL_PZGiK_994_BDOT10k_0463__OT_SKJZ_L.shp'
    cache_path = os.path.join(os.path.dirname(__file__), "static\graph_cache.json")


    g = load_graph(cache_path)
    if g is None:
        g = create_graph(workspace, layer)
        save_graph(g, cache_path)

    
    a = astar(g, 1000, 6767)
    path_nodes, path_edges, _, _, _ = a

