import dash_bootstrap_components as dbc
from dash import dcc, html

# Define the layout as a function (for reusability)
def create_layout():
    return dbc.Container([
        html.H1("Real-Time IMU Data", className="text-center text-dark mb-4"),
        dcc.Store(id='plotting-state', data={'running': True}),  # Track plotting state
        dbc.Row([
            dbc.Col(dcc.Graph(id='imu-graph-accel'), width=6),
            dbc.Col(dcc.Graph(id='imu-graph-gyro'), width=6)
        ]),
        dcc.Graph(id='rocket-orientation'),
        dcc.Interval(id='interval-component', interval=100, n_intervals=0)
    ], fluid=True)

