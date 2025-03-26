import dash
from dash.dependencies import Input, Output
import dash_bootstrap_components as dbc
import plotly.graph_objs as go
import serial
from collections import deque
from layout import create_layout
from callbacks import register_callbacks
from dash_extensions import WebSocket

# Initialize the Dash app
app = dash.Dash(__name__, external_stylesheets=[dbc.themes.CYBORG])
WebSocket(url="ws://127.0.0.1:5000", id="ws")

# Create a serial connection (update the port as needed)
ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)

# Define layout with an Interval component
app.layout = create_layout()

# Register callbacks
register_callbacks(app)
 
# Run the Dash app
if __name__ == '__main__':
    app.run(debug=True)
