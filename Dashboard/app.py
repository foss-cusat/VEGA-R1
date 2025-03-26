import dash
from dash import dcc, html
from dash.dependencies import Input, Output
import plotly.graph_objs as go
import serial
from collections import deque
import numpy as np

app = dash.Dash(__name__)

ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)

app.layout = html.Div([
    dcc.Graph(id='imu-graph'),
    dcc.Interval(
        id='interval-component',
        interval=1000, 
        n_intervals=0
    )
])

window_size = 10
accel_history = deque(maxlen=window_size)
gyro_history = deque(maxlen=window_size)

@app.callback(
    Output('imu-graph', 'figure'),
    Input('interval-component', 'n_intervals')  
)
def update_graph(n_intervals):
    try:
        line = ser.readline().decode('utf-8').strip()
        values = [float(x) for x in line.split(',')]  

        accel_history.append(values[:3])  
        gyro_history.append(values[3:])   

        smooth_accel = np.mean(accel_history, axis=0)
        smooth_gyro = np.mean(gyro_history, axis=0)

        fig = go.Figure()
        fig.add_trace(go.Scatter(y=smooth_accel, mode='lines', name='Accelerometer'))
        fig.add_trace(go.Scatter(y=smooth_gyro, mode='lines', name='Gyroscope'))

        return fig
    except Exception as e:
        return go.Figure()  

if __name__ == '__main__':
    app.run(debug=True)
