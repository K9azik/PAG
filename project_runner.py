import os
import dotenv
import arcpy
from main import GraphCreator, create_graph, load_graph, save_graph, astar

def convert_roads_geojson(workspace, workspace_name, output_path):
    if os.path.exists(output_path):
        print(f"GeoJSON already exists: {output_path}")
        return
    
    input_path = os.path.join(workspace, workspace_name)
    arcpy.conversion.FeaturesToJSON(
        in_features=input_path,
        out_json_file=output_path,
        format_json="FORMATTED",
        geoJSON="GEOJSON",
        outputToWGS84="WGS84",
    )
    print(f"GeoJSON created: {output_path}")

def save_to_geojson(workspace, layer, path_edges, output_path):  
    if os.path.exists(output_path):
        os.remove(output_path)
    
    # Sprawdź czy są jakieś krawędzie
    if not path_edges:
        # Utwórz pusty GeoJSON
        empty_geojson = {
            "type": "FeatureCollection",
            "features": []
        }
        import json
        with open(output_path, 'w') as f:
            json.dump(empty_geojson, f)
        return output_path
    
    temp_layer = "temp_layer"
    
    where_clause = f"FID IN ({','.join(map(str, path_edges))})"
    arcpy.MakeFeatureLayer_management(
        in_features=os.path.join(workspace, layer),
        out_layer=temp_layer,
        where_clause=where_clause
    )

    arcpy.conversion.FeaturesToJSON(
        in_features=temp_layer,
        out_json_file=output_path,
        format_json="FORMATTED",
        geoJSON="GEOJSON",
        outputToWGS84="WGS84"
    )
    
    arcpy.Delete_management(temp_layer)
    
    return output_path

def format_time(seconds):
        hours = int(seconds // 3600)
        minutes = int((seconds % 3600) // 60)
        seconds = int(seconds % 60)
        
        if hours > 0:
            time_formatted = f"{hours}h {minutes}m {seconds}s"
        elif minutes > 0:
            time_formatted = f"{minutes}m {seconds}s"
        else:
            time_formatted = f"{seconds}s"
        
        return time_formatted

class Runner:
    def __init__(self):
        dotenv.load_dotenv()
        
        self.gc = GraphCreator()
        self.workspace = r'C:\uni\5sem\pagdane\drogi\PL_PZGiK_994_BDOT10k_0463__OT_SKJZ_L.shp'
        self.workspace_name = 'PL_PZGiK_994_BDOT10k_0463__OT_SKJZ_L.shp'
        
        base_dir = os.path.dirname(os.path.abspath(__file__))
        roads_geojson_path = os.path.join(base_dir, 'static', 'roads.geojson')
        convert_roads_geojson(self.workspace, self.workspace_name, roads_geojson_path)
    
        cache_path = os.path.join(base_dir, 'static\graph_cache.json')
        graph = load_graph(cache_path)
        if graph is None:
            graph = create_graph(self.workspace, self.workspace_name)
            save_graph(graph, cache_path)
        
        self.gc.graph = graph
      
        self.workspace = self.workspace
        self.workspace_name = self.workspace_name
       
        self.output_path = os.path.join(base_dir, 'static', 'shortest_path.geojson')

    def find_closest_points(self, point1, point2):
        start_node_id, end_node_id = self.gc.find_closest_nodes(point1, point2)
        return start_node_id, end_node_id

    def calculate_path(self, start_id, end_id):
        _, path_edges, g_cost, _, _ = astar(self.gc.graph, start_id, end_id)

        
        result_path = save_to_geojson(
            self.workspace,
            self.workspace_name,
            path_edges,
            self.output_path
        )
        
        g_cost = format_time(g_cost)
        return g_cost, result_path