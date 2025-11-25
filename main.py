import arcpy
import math
import os
import dotenv
from arcpy import SpatialReference

dotenv.load_dotenv()

#####słownik do heurystyki#####
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


class Node:
    def __init__(self, node_id, x, y):
        self.id = node_id  # id węzła
        self.x = x  # współrzędna x
        self.y = y  # współrzędna y
        self.edges = []  # lista krawędzi (krawędź, wierzchołek)

    def __repr__(self):
        eids = ",".join([str(e.id) for e, v in self.edges])
        return f"N({self.id},({self.x},{self.y}),[{eids}])"


class Edge:
    def __init__(self, edge_id, cost, start, end, road_class):
        self.id = edge_id  # nr krawędzi
        self.cost = cost  # waga/długość
        self.start = start  # początek
        self.end = end  # koniec
        self.road_class = road_class

    def __repr__(self):
        sid = self.start.id if self.start is not None else "None"
        eid = self.end.id if self.end is not None else "None"
        return f"E({self.id},{sid},{eid})"


class Graph:
    def __init__(self):
        self.edges = {}  # edge_id -> Edge
        self.nodes = {}  # node_id -> Node

    def __repr__(self):
        ns = ",".join([str(n) for n in self.nodes.values()])  
        es = ",".join([str(e) for e in self.edges.values()])  
        return f"Graph:\n  Nodes: [{ns}]\n  Edges: [{es}]"


class GraphCreator:
    def __init__(self):
        self.graph = Graph()  # graf
        self.new_id = 0  # licznik id
        self.index = {}  # indeks punktów

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
    # init
    S = set()  # odwiedzone
    Q = set()  # do sprawdzenia
    d = {}  # dystanse
    p = {}  # poprzednicy
    pe = {}  # poprzednie krawędzie
    neighbor_count = 0  # licznik sąsiadów

    # ustaw odległości początkowe
    for node_id in graph.nodes:
        d[node_id] = math.inf
        p[node_id] = None
        pe[node_id] = None

    d[start_id] = 0
    Q.add(start_id)

    # --- Pętla główna ---
    while Q:
        # wybierz wierzchołek o najmniejszym d[v]
        v = min(Q, key=lambda node_id: d[node_id])
        Q.remove(v)

        if v == end_id:
            break

        v_node = graph.nodes[v]

        # przejrzyj wszystkich sąsiadów
        for edge, u_node in v_node.edges:
            neighbor_count += 1
            u = u_node.id
            if u in S:
                continue

            new_dist = d[v] + edge.cost
            if new_dist < d[u]:
                d[u] = new_dist
                p[u] = v
                pe[u] = edge
                Q.add(u)

        S.add(v)

    # --- Odtworzenie ścieżki ---
    path_nodes = []
    path_edges = []

    if d[end_id] < math.inf:
        node = end_id
        while node is not None:
            path_nodes.insert(0, node)
            edge = pe[node]
            if edge is not None:
                path_edges.insert(0, edge.id)
            node = p[node]
    else:
        print("Brak połączenia między wierzchołkami.")
        return None, None, None, None

    # --- Wyniki ---
    # print(f"Najkrotsza odleglosc od {start_id} do {end_id}: {d[end_id]:.2f}")
    # print(f"Sciezka po wierzcholkach: {path_nodes}")
    # print(f"Krawedzie trasy: {path_edges}")
    # print(f"Liczba odwiedzonych wierzcholkow: {len(S)}")
    # print(f"Liczba przejrzanych sasiadow: {neighbor_count}")

    return path_nodes, path_edges, d[end_id], len(S), neighbor_count


def astar(graph, start_id, end_id):

    def heuristic(n1, n2, prev_edge=None):
        dx = n1.x - n2.x
        dy = n1.y - n2.y
        distance = math.sqrt(dx * dx + dy * dy)

        vmax = max(Vs.values()) / 3.6
        time = distance / vmax

        ####koszt (czas) wzrasta przy skrecaniu w zaleznosci od kąta
        turn_penalty = 0
        if prev_edge is not None:
            vx1 = prev_edge.end.x - prev_edge.start.x
            vy1 = prev_edge.end.y - prev_edge.start.y
            vx2 = n2.x - n1.x
            vy2 = n2.y - n1.y

            # iloczyn skalarny i dlugosci wektorow
            dot = vx1 * vx2 + vy1 * vy2
            mag1 = math.sqrt(vx1 ** 2 + vy1 ** 2)
            mag2 = math.sqrt(vx2 ** 2 + vy2 ** 2)
            if mag1 > 0 and mag2 > 0:
                angle = math.degrees(math.acos(max(-1, min(1, dot / (mag1 * mag2)))))
                turn_penalty = (angle / 180) * time

        ####koszt za klase drogi
        road_penalty = 0
        if prev_edge is not None:
            v_edge = Vs.get(prev_edge.road_class, 40) / 3.6
            road_penalty = (1 - v_edge / vmax) * time

        return time + turn_penalty + road_penalty


    S = set()  # odwiedzone
    Q = set()  # do sprawdzenia
    g_cost = {}  # koszt od startu do danego wierzchołka
    f_cost = {}  # koszt całkowity (g + h)
    p = {}  # poprzednicy
    pe = {}  # poprzednie krawędzie
    neighbor_count = 0

    for node_id in graph.nodes:
        g_cost[node_id] = math.inf
        f_cost[node_id] = math.inf
        p[node_id] = None
        pe[node_id] = None

    start_node = graph.nodes[start_id]
    end_node = graph.nodes[end_id]

    g_cost[start_id] = 0
    f_cost[start_id] = heuristic(start_node, end_node, None)
    Q.add(start_id)

    while Q:
        # wybierz wierzchołek o najmniejszym f_cost
        v = min(Q, key=lambda node_id: f_cost[node_id])
        Q.remove(v)

        if v == end_id:
            break

        v_node = graph.nodes[v]
        S.add(v)

        # przeglądaj sąsiadów
        for edge, u_node in v_node.edges:
            neighbor_count += 1
            u = u_node.id

            if u in S:
                continue

            tentative_g = g_cost[v] + edge.cost
            if tentative_g < g_cost[u]:
                g_cost[u] = tentative_g
                f_cost[u] = tentative_g + heuristic(u_node, end_node, pe[v])
                p[u] = v
                pe[u] = edge
                Q.add(u)

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
        print("Brak połączenia między wierzchołkami.")
        return None, None, None, None, None

    # --- Wyniki ---
    print(f"[A*] Najkrotsza odleglosc od {start_id} do {end_id}: {g_cost[end_id]:.2f}")
    print(f"[A*] Sciezka po wierzcholkach: {path_nodes}")
    print(f"[A*] Krawedzie trasy: {path_edges}")
    print(f"[A*] Liczba odwiedzonych wierzcholkow: {len(S)}")
    print(f"[A*] Liczba przejrzanych sasiadow: {neighbor_count}")

    return path_nodes, path_edges, g_cost[end_id], len(S), neighbor_count


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
    if not os.path.exists(path):
        print("Plik grafu nie istnieje – zostanie utworzony nowy.")
        return None

    with open(path, "r", encoding="utf-8") as f:
        data = json.load(f)

    g = Graph()
    # Odtwarzanie węzłów
    for nid, ndata in data["nodes"].items():
        n = Node(int(nid), ndata["x"], ndata["y"])
        g.nodes[n.id] = n

    # Odtwarzanie krawędzi
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

    ###do testowania w arcgisie
    '''
    fid_query = f"FID IN ({','.join(map(str, path_edges))})"
    aprx = arcpy.mp.ArcGISProject("CURRENT")
    m = aprx.activeMap
    m.clearSelection()

    layer_obj = layer
    arcpy.management.SelectLayerByAttribute(layer_obj, "NEW_SELECTION", fid_query)
    '''

#gdyby ktos sie zapytal skad wzielismy predkosci do heurystyki:
#https://sip.lex.pl/akty-prawne/dzu-dziennik-ustaw/przepisy-techniczno-budowlane-dotyczace-drog-publicznych-19262089/dz-3-roz-1
