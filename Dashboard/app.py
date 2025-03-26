import dash
from dash import dcc, html
from dash.dependencies import Input, Output
import plotly.graph_objs as go
import serial

# Initialize the Dash app
app = dash.Dash(__name__)

# Create a serial connection (update the port as needed)
ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)

# Define layout with an Interval component
app.layout = html.Div([
    dcc.Graph(id='imu-graph'),
    dcc.Interval(
        id='interval-component',
        interval=1000,  # Update every 1000 ms (1 second)
        n_intervals=0
    )
])

# Define callback with an `Input` element
@app.callback(
    Output('imu-graph', 'figure'),
    Input('interval-component', 'n_intervals')  # Triggers the update
)
def update_graph(n_intervals):
    try:
        line = ser.readline().decode('utf-8').strip()  # Read IMU data from serial
        values = [float(x) for x in line.split(',')]  # Assuming CSV format (ax, ay, az, gx, gy, gz)
        
        # Create a graph with accelerometer & gyroscope data
        fig = go.Figure()
        fig.add_trace(go.Scatter(y=values[:3], mode='lines', name='Accelerometer'))
        fig.add_trace(go.Scatter(y=values[3:], mode='lines', name='Gyroscope'))
        
        return fig
    except Exception as e:
        return go.Figure()  # Return an empty figure on error

# Run the Dash app
if __name__ == '__main__':
    app.run(debug=True)
