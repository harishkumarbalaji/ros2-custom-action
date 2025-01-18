from flask import Flask, jsonify, request
import json

app = Flask(__name__)

# Initialize the mission
mission = {
    'id': 1,
    'description': 'Go to packaging area',
}

print(type(mission))

# Handle mission update POST requests
@app.route('/update_mission', methods=['POST'])
def update_record():
    """
    Update the mission with the data in the request body
    """
    global mission
    mission = json.loads(request.data)
    print('Recieved mission')
    print(mission)
    # return mission updated status
    return jsonify({'mission': mission})

# Handle mission retrieval GET requests
@app.route('/get_mission', methods=['GET'])
def get_mission():
    """
    Return the current mission
    """
    global mission
    return jsonify({'mission': mission})

if __name__ == '__main__':
    # Run the server
    app.run(host='0.0.0.0', port=5000)
