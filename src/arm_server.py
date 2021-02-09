from flask import Flask, render_template
from flask_socketio import SocketIO, emit
from flask_cors import CORS

from init_virtual import setup

app = Flask(__name__)
app.config['SECRET_KEY'] = 'secret!'
CORS(app)
socketio = SocketIO(app)

arm = setup()



#serves html
@app.route('/')
def load_dashboard():
    return render_template('index.html')

@socketio.on('get_update')
def update_arm():
    arm.update()
    data = arm.export_data()
    emit('update', data)

@socketio.on('jog')
def jog(data):
    amount = float(data['amount'])
    packet = {
        't1': 0,
        't2': 0,
        't3': 0,
        'x': 0,
        'y': 0,
        'z': 0,
    }
    packet[data['axis']] += amount
    packet= list(packet.values())

    arm.jog(packet[:3], packet[3:])


if __name__ == '__main__':
    print('[INFO] Starting server at http://localhost:6969')
    socketio.run(app=app, host='0.0.0.0', port=6969, debug=False)