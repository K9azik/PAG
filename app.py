from flask import Flask, request, jsonify, send_from_directory
import os
from flask_cors import CORS
from project_runner import Runner

app = Flask(__name__, static_url_path='', static_folder='static')
CORS(app)

# Global runner
runner = None

def initialize_app():
    global runner
    runner = Runner()

@app.route('/')
def serve_index():
    return send_from_directory('static', 'index.html')

@app.route('/process', methods=['POST'])
def process():
    data = request.get_json()

    point1 = (float(data['p1']['lng']), float(data['p1']['lat']))
    point2 = (float(data['p2']['lng']), float(data['p2']['lat']))
    
    start_node, end_node = runner.find_closest_points(point1, point2)

    if start_node == end_node:
        return jsonify({
            "status": "error",
            "message": "Start and end points are too close or the same"
        }), 400
    
    distance, geojson_path = runner.calculate_path(start_node, end_node)
    
    if geojson_path is None:
        return jsonify({
            "status": "error",
            "message": "No path found between selected points"
        }), 404
    
    return jsonify({
        "status": "success",
        "geojson_url": "/shortest_path.geojson",
        "start_node": start_node,
        "end_node": end_node,
        "distance": distance
    })

if __name__ == '__main__':
    initialize_app()
    app.run(debug=True, use_reloader=False)
