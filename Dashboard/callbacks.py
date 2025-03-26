from dash.dependencies import Input, Output
import plotly.graph_objs as go
import numpy as np
from collections import deque
import serial

# Connect to Serial Port
ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)

# Data storage for smoothing
window_size = 10
accel_history = np.zeros((10, 3))
gyro_history = np.zeros((10, 3))


def register_callbacks(app):
    @app.callback(
        [Output('imu-graph-accel', 'figure'),
         Output('imu-graph-gyro', 'figure')],
        [Input('interval-component', 'n_intervals')]
    )
    def update_graph(n_intervals):
        try:
            # Read from serial
            line = ser.readline().decode('utf-8').strip()
            values = [float(x) for x in line.split(',')]

            # Shift old values left
            global accel_history, gyro_history
            accel_history[:-1] = accel_history[1:]  # Shift up (drop oldest)
            gyro_history[:-1] = gyro_history[1:]

            # Insert new values at the end
            accel_history[-1] = values[:3]
            gyro_history[-1] = values[3:]

            # Acceleration Figure
            accel_fig = go.Figure()
            accel_fig.add_trace(go.Scatter(y=accel_history[:, 0], mode='lines', name='Ax (m/s²)', line=dict(color='red')))
            accel_fig.add_trace(go.Scatter(y=accel_history[:, 1], mode='lines', name='Ay (m/s²)', line=dict(color='green')))
            accel_fig.add_trace(go.Scatter(y=accel_history[:, 2], mode='lines', name='Az (m/s²)', line=dict(color='blue')))

            accel_fig.update_layout(
                title="Acceleration Data",
                xaxis_title="Time (Last 10 Samples)",
                yaxis_title="Acceleration (m/s²)",
                legend_title="Legend",
                template="plotly_dark"
            )

            # Gyroscope Figure
            gyro_fig = go.Figure()
            gyro_fig.add_trace(go.Scatter(y=gyro_history[:, 0], mode='lines', name='Gx (°/s)', line=dict(color='orange')))
            gyro_fig.add_trace(go.Scatter(y=gyro_history[:, 1], mode='lines', name='Gy (°/s)', line=dict(color='purple')))
            gyro_fig.add_trace(go.Scatter(y=gyro_history[:, 2], mode='lines', name='Gz (°/s)', line=dict(color='cyan')))

            gyro_fig.update_layout(
                title="Gyroscope Data",
                xaxis_title="Time (Last 10 Samples)",
                yaxis_title="Angular Velocity (°/s)",
                legend_title="Legend",
                template="plotly_dark"
            )

            return accel_fig, gyro_fig

        except Exception as e:
            return go.Figure(), go.Figure()  # Return empty figures if there's an error

