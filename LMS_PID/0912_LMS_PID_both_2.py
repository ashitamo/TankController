import dash
from dash import dcc, html
from dash.dependencies import Input, Output, State
import serial
import time

# Serial Port Information
SERIAL_PORT = '/dev/tty.usbmodem1201'  # Update this to your port
BAUD_RATE = 57600

# Global Serial Object
arduino = None

# Function to open/reopen the serial connection
def open_serial_connection():
    global arduino
    while True:
        try:
            arduino = serial.Serial(port=SERIAL_PORT, baudrate=BAUD_RATE, timeout=1)
            time.sleep(2)  # Allow time for the connection to stabilize
            print("Serial connection established")
            return
        except serial.SerialException:
            print("Failed to connect. Retrying in 2 seconds...")
            time.sleep(2)

# Attempt to open serial connection initially
open_serial_connection()

# Create Dash App
app = dash.Dash(__name__)
app.title = "Arduino Dual Motor Control Dashboard（灌和連接時，要斷電源）(初始未移動過時，請勿進行Sw Reset)"

app.layout = html.Div([
    html.H1("Arduino Dual Motor Control Dashboard（灌和連接時，要斷電源）(初始未移動過時，請勿進行Sw Reset)"),

    # ===============================
    # Controls for BASE Motor
    # ===============================
    html.Div([
        html.H2("BASE Motor Control"),
        # Input and Set Button
        dcc.Input(id='base-angle-input', type='number', placeholder='Enter BASE target angle', value=0),
        html.Button('Set BASE Angle', id='set-base-angle-button', n_clicks=0),
        html.Br(),
        # Increase and Decrease Buttons
        html.Button('Increase BASE Angle 砲管向右', id='increase-base-angle-button', n_clicks=0),
        html.Button('Decrease BASE Angle 砲管向左', id='decrease-base-angle-button', n_clicks=0),
        html.Br(),
        # Reset Button
        html.Button('Reset BASE Motor', id='reset-base-button', n_clicks=0),
        # Reset BASE Motor Switch
        html.Button('Reset BASE Motor Switch', id='reset-ms-base-button', n_clicks=0),
        html.Br(),
        # Display Current Angle
        html.H3("BASE Current Angle:"),
        html.Div(id='base-current-angle-display', style={'fontSize': 24}),
    ], style={'border': '1px solid black', 'padding': '10px', 'margin': '10px'}),

    # ===============================
    # Controls for FORT Motor
    # ===============================
    html.Div([
        html.H2("FORT Motor Control"),
        # Input and Set Button
        dcc.Input(id='fort-angle-input', type='number', placeholder='Enter FORT target angle', value=0),
        html.Button('Set FORT Angle', id='set-fort-angle-button', n_clicks=0),
        html.Br(),
        # Increase and Decrease Buttons
        html.Button('Increase FORT Angle 砲管向下', id='increase-fort-angle-button', n_clicks=0),
        html.Button('Decrease FORT Angle 砲管向上', id='decrease-fort-angle-button', n_clicks=0),
        html.Br(),
        # Reset Button
        html.Button('Reset FORT Motor', id='reset-fort-button', n_clicks=0),
        # Reset FORT Motor Switch
        html.Button('Reset FORT Motor Switch', id='reset-ms-fort-button', n_clicks=0),
        html.Br(),
        # Display Current Angle
        html.H3("FORT Current Angle:"),
        html.Div(id='fort-current-angle-display', style={'fontSize': 24}),
    ], style={'border': '1px solid black', 'padding': '10px', 'margin': '10px'}),

    # Interval Component for Updating Angles
    dcc.Interval(id='interval-component', interval=500, n_intervals=0)
])

# ===============================
# Serial Communication Functions with Reconnect Logic
# ===============================
def send_serial_command(command):
    """Send command to Arduino via serial and handle disconnection."""
    global arduino
    try:
        if arduino is None or not arduino.is_open:
            open_serial_connection()
        arduino.write(command.encode())
        time.sleep(0.01)
    except serial.SerialException:
        print("Serial connection lost. Reconnecting...")
        open_serial_connection()
        arduino.write(command.encode())  # Resend the command after reconnect

def get_current_angle(motor):
    """Request and retrieve the current angle for the specified motor."""
    command = f"GET_{motor}\n"
    send_serial_command(command)
    #time.sleep(0.01)
    print(motor)
    try:
        while arduino.in_waiting:
            response = arduino.readline().decode('utf-8', errors='ignore').strip()
            print(response)
            if motor == "FORT" and response.startswith("FORT_ANGLE:"):
                return float(response.split(":")[1])
            if motor == "BASE" and response.startswith("BASE_ANGLE:"):
                print('send')
                return float(response.split(":")[1])
                
    except serial.SerialException:
        print(f"Failed to retrieve {motor} angle. Reconnecting...")
        open_serial_connection()
        return None
    
def set_target_angle(motor, angle):
    command = f"SET_{motor}:{angle}\n"
    send_serial_command(command)

def reset_motor(motor):
    command = f"RESET_{motor}\n"
    send_serial_command(command)

def reset_motor_switch(motor):
    command = f"RESET_MS_{motor}\n"
    send_serial_command(command)
    
    # Wait for the Arduino to reset or stabilize
    time.sleep(5)  # Adjust this delay based on how long the reset takes

    # Try reconnecting the serial connection
    for _ in range(5):  # Retry 5 times
        try:
            open_serial_connection()
            print(f"Reconnected to {motor} after reset.")
            break
        except serial.SerialException:
            print(f"Failed to reconnect to {motor}. Retrying...")
            time.sleep(2)


# ===============================
# Callback Function for BASE and FORT Control
# ===============================
@app.callback(
    [
        Output('base-current-angle-display', 'children'),
        Output('fort-current-angle-display', 'children'),
        Output('base-angle-input', 'value'),
        Output('fort-angle-input', 'value')
    ],
    [
        Input('interval-component', 'n_intervals'),
        Input('set-base-angle-button', 'n_clicks'),
        Input('increase-base-angle-button', 'n_clicks'),
        Input('decrease-base-angle-button', 'n_clicks'),
        Input('reset-base-button', 'n_clicks'),
        Input('reset-ms-base-button', 'n_clicks'),
        Input('set-fort-angle-button', 'n_clicks'),
        Input('increase-fort-angle-button', 'n_clicks'),
        Input('decrease-fort-angle-button', 'n_clicks'),
        Input('reset-fort-button', 'n_clicks'),
        Input('reset-ms-fort-button', 'n_clicks')
    ],
    [
        State('base-angle-input', 'value'),
        State('fort-angle-input', 'value')
    ]
)
def update_angles_display(n_intervals, 
                          set_base_clicks, inc_base_clicks, dec_base_clicks, reset_base_clicks, reset_ms_base_clicks,
                          set_fort_clicks, inc_fort_clicks, dec_fort_clicks, reset_fort_clicks, reset_ms_fort_clicks,
                          base_angle_input, fort_angle_input):
    ctx = dash.callback_context
    button_id = ctx.triggered[0]['prop_id'].split('.')[0] if ctx.triggered else None

    # Initialize or update target angles for BASE
    if button_id == 'set-base-angle-button' and set_base_clicks > 0:
        set_target_angle("BASE", base_angle_input)
    elif button_id == 'increase-base-angle-button' and inc_base_clicks > 0:
        base_angle_input += 5
        set_target_angle("BASE", base_angle_input)
    elif button_id == 'decrease-base-angle-button' and dec_base_clicks > 0:
        base_angle_input -= 5
        set_target_angle("BASE", base_angle_input)
    elif button_id == 'reset-base-button' and reset_base_clicks > 0:
        reset_motor("BASE")
        base_angle_input = 0
    elif button_id == 'reset-ms-base-button' and reset_ms_base_clicks > 0:
        reset_motor_switch("BASE")
        base_angle_input = 0

    # Initialize or update target angles for FORT
    if button_id == 'set-fort-angle-button' and set_fort_clicks > 0:
        set_target_angle("FORT", fort_angle_input)
    elif button_id == 'increase-fort-angle-button' and inc_fort_clicks > 0:
        fort_angle_input += 5
        set_target_angle("FORT", fort_angle_input)
    elif button_id == 'decrease-fort-angle-button' and dec_fort_clicks > 0:
        fort_angle_input -= 5
        set_target_angle("FORT", fort_angle_input)
    elif button_id == 'reset-fort-button' and reset_fort_clicks > 0:
        reset_motor("FORT")
        fort_angle_input = 0
    elif button_id == 'reset-ms-fort-button' and reset_ms_fort_clicks > 0:
        reset_motor_switch("FORT")

    # Get current angles for both motors
   
    
    fort_current_angle = get_current_angle("FORT")
    base_current_angle = get_current_angle("BASE")
    print(base_current_angle, fort_current_angle)

    base_display = f"{base_current_angle:.2f} degrees" if base_current_angle is not None else "No data"
    fort_display = f"{fort_current_angle:.2f} degrees" if fort_current_angle is not None else "No data"

    return base_display, fort_display, base_angle_input, fort_angle_input

# ===============================
# Run the Dash App (Keeps the server running)
# ===============================
if __name__ == '__main__':
    app.run_server(debug=True, port=8051, use_reloader=False)
