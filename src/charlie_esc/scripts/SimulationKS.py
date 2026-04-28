import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib.patches as patches

# ==========================================
# 1. Vehicle Parameters and State
# ==========================================
dt = 0.05
L = 2.391           # Wheelbase [m]
max_steer = np.radians(35) 
max_speed = 5.0     # m/s
a_max = 3.0         # Maximum friction acceleration [m/s^2] (Friction Circle limit)

state = {
    'x': 0.0, 'y': 0.0, 'psi': 0.0, 
    'v': 0.0, 'delta': 0.0
}
history_x, history_y = [], []
keys = {'up': False, 'down': False, 'left': False, 'right': False}
friction_warning = False # Flag for the HUD

# Keyboard events
def on_press(event):
    if event.key in ['up', 'w']: keys['up'] = True
    if event.key in ['down', 's']: keys['down'] = True
    if event.key in ['left', 'a']: keys['left'] = True
    if event.key in ['right', 'd']: keys['right'] = True

def on_release(event):
    if event.key in ['up', 'w']: keys['up'] = False
    if event.key in ['down', 's']: keys['down'] = False
    if event.key in ['left', 'a']: keys['left'] = False
    if event.key in ['right', 'd']: keys['right'] = False

# ==========================================
# 2. Visual Configuration (Technical Style)
# ==========================================
plt.style.use('default') 
fig, ax = plt.subplots(figsize=(10, 8))
fig.canvas.manager.set_window_title('Kinematic Single Track Simulation')
fig.canvas.mpl_connect('key_press_event', on_press)
fig.canvas.mpl_connect('key_release_event', on_release)

ax.set_aspect('equal')
ax.grid(True, linestyle=':', color='gray', alpha=0.5)

# Diagram elements
trail, = ax.plot([], [], color='blue', alpha=0.3, linewidth=2, label='Path ($s_x, s_y$)')
chassis_line, = ax.plot([], [], color='black', linewidth=6, zorder=2, label='Chassis ($l_{wb}$)')

# Wheel patches
wheel_len, wheel_width = 1.0, 0.45 
rear_wheel = patches.Polygon(np.zeros((4,2)), facecolor='black', alpha=0.9, zorder=3)
front_wheel = patches.Polygon(np.zeros((4,2)), facecolor='red', alpha=0.9, zorder=3)

ax.add_patch(rear_wheel)
ax.add_patch(front_wheel)

# Telemetry HUD
hud_text = ax.text(0.02, 0.98, '', transform=ax.transAxes, fontsize=11, 
                   family='serif', verticalalignment='top',
                   bbox=dict(facecolor='white', alpha=0.9, edgecolor='gray'))

# ==========================================
# 3. Transformation Functions
# ==========================================
def get_wheel_shape():
    """Base shape of the wheel centered at the origin."""
    return np.array([
        [-wheel_len/2, -wheel_width/2],
        [wheel_len/2, -wheel_width/2],
        [wheel_len/2,  wheel_width/2],
        [-wheel_len/2,  wheel_width/2]
    ])

def transform(pts, x, y, angle):
    """Applies rotation and translation to the polygons."""
    R = np.array([[np.cos(angle), -np.sin(angle)], 
                  [np.sin(angle),  np.cos(angle)]])
    return (R @ pts.T).T + np.array([x, y])

wheel_base_shape = get_wheel_shape()

# ==========================================
# 4. Simulation and Animation Loop
# ==========================================
def update(frame):
    global friction_warning
    
    # --- A. Control Inputs (u1 and u2) ---
    # u2: Longitudinal Acceleration (a_long)
    if keys['up']: u2 = 2.0
    elif keys['down']: u2 = -2.0
    else: 
        # Rolling friction to slow down smoothly
        u2 = -0.5 * np.sign(state['v']) if abs(state['v']) > 0.1 else 0.0 

    # u1: Steering Velocity (v_delta)
    if keys['left']: u1 = 1.5
    elif keys['right']: u1 = -1.5
    else: 
        # Auto-center steering wheel
        u1 = -2.5 * state['delta'] 

    # --- B. KS Equations (Eq 6 & 8 from paper) ---
    
    # Eq 6: Heading derivative (psi_dot)
    psi_dot = (state['v'] / L) * np.tan(state['delta'])
    
    # Eq 8: Friction Circle Constraint 
    # sqrt( u2^2 + (x4 * x5_dot)^2 ) <= a_max
    acc_lat = state['v'] * psi_dot
    acc_total = np.sqrt(u2**2 + acc_lat**2)

    if acc_total > a_max:
        friction_warning = True
        # Scale down inputs to strictly respect the physical limit
        scale = a_max / acc_total
        u2 *= scale
        psi_dot *= scale 
    else:
        friction_warning = False

    # Integrations (Euler Method)
    state['v'] += u2 * dt
    state['v'] = np.clip(state['v'], -max_speed, max_speed)
    
    state['delta'] += u1 * dt
    state['delta'] = np.clip(state['delta'], -max_steer, max_steer)
    
    state['x'] += state['v'] * np.cos(state['psi']) * dt
    state['y'] += state['v'] * np.sin(state['psi']) * dt
    state['psi'] += psi_dot * dt

    history_x.append(state['x'])
    history_y.append(state['y'])
    trail.set_data(history_x, history_y)

    # --- C. Graphics Update ---
    rx, ry, psi = state['x'], state['y'], state['psi'] 
    fx = rx + L * np.cos(psi) 
    fy = ry + L * np.sin(psi)
    
    chassis_line.set_data([rx, fx], [ry, fy])
    rear_wheel.set_xy(transform(wheel_base_shape, rx, ry, psi))
    front_wheel.set_xy(transform(wheel_base_shape, fx, fy, psi + state['delta']))

    # --- D. Camera and HUD ---
    margin = 7.0 
    ax.set_xlim(rx - margin, rx + margin)
    ax.set_ylim(ry - margin, ry + margin)

    R_turning = (L / np.abs(np.tan(state['delta']))) if abs(state['delta']) > 0.02 else float('inf')
    
    warning_str = "FRICTION LIMIT EXCEEDED!" if friction_warning else ""
    
    hud_info = (f"KS SIMULATION: Arrows / WASD\n"
                f"---------------------------\n"
                f"Velocity (v): {state['v']:.2f} m/s\n"
                f"Wheel Angle (delta): {np.degrees(state['delta']):.1f}°\n"
                f"Heading (psi): {np.degrees(state['psi']):.1f}°\n"
                f"Turning Radius (R): {R_turning:.2f} m\n"
                f"Pos X, Y: {state['x']:.1f}, {state['y']:.1f} \n\n"
                f"{warning_str}")
                
    # Change box color to light red if slipping
    box_color = '#ffe6e6' if friction_warning else 'white'
    hud_text.set_text(hud_info)
    hud_text.set_bbox(dict(facecolor=box_color, alpha=0.9, edgecolor='gray'))

    return trail, chassis_line, rear_wheel, front_wheel, hud_text

ani = animation.FuncAnimation(fig, update, interval=dt*1000, blit=False, cache_frame_data=False)
ax.legend(loc='lower right', fontsize=10)
plt.show()