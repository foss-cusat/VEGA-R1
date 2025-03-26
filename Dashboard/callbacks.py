from dash.dependencies import Input, Output, State
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

roll, pitch, yaw = 0, 0, 0

def register_callbacks(app):
    @app.callback(
        [Output('imu-graph-accel', 'figure'),
         Output('imu-graph-gyro', 'figure')],
         Output('rocket-orientation', 'figure'),       
        [Input('interval-component', 'n_intervals')],
        [State("plotting-state", "data")]
    )
    def update_graph(n_intervals, plotting_state):
        global roll, pitch, yaw

        if not plotting_state["running"]:  # If paused, return empty figures
            return go.Figure(), go.Figure(), go.Figure()
        
        try:
            # Read from serial
            line = ser.readline().decode('utf-8').strip()
            values = [float(x) for x in line.split(',')]

            accel_data = values[:3]  # Acceleration Data
            gyro_data = values[3:]   # Gyroscope Data (degrees/s)

            # Integrate gyro data to estimate orientation
            dt = 0.2  # Time step (adjust based on actual IMU update rate)
            roll += gyro_data[0] * dt
            pitch += gyro_data[1] * dt
            yaw += gyro_data[2] * dt

            # Ensure angles stay within -180 to 180 degrees
            roll = (roll + 180) % 360 - 180
            pitch = (pitch + 180) % 360 - 180
            yaw = (yaw + 180) % 360 - 180
            
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

            # 3D Rocket Orientation Plot
            rocket_fig = go.Figure()
        
            # Rocket body as a cone
            rocket_fig.add_trace(go.Cone(
                x=[0], y=[0], z=[0],  
                u=[np.cos(np.radians(yaw))],  
                v=[np.sin(np.radians(yaw))],  
                w=[np.sin(np.radians(pitch))],  
                colorscale='Blues',
                sizemode="absolute",
                sizeref=1
            ))

            rocket_fig.update_layout(
                title="Rocket Orientation",
                scene=dict(
                    xaxis=dict(title="X"),
                    yaxis=dict(title="Y"),
                    zaxis=dict(title="Z")
                ),
                template="plotly_dark"
            )

            return accel_fig, gyro_fig, rocket_fig

        except Exception as e:
            return go.Figure(), go.Figure(), go.Figure()
