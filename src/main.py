from vpython import *
from envir import RobotEnvironment
from slam import SLAM
import numpy as np
import matplotlib.pyplot as plt
import csv
import os
from datetime import datetime

# Initialize environment and SLAM
env = RobotEnvironment()
slam = SLAM(env)

# Control variables
manual_mode = True
dx, dy, dtheta = 0, 0, 0
path = []
returned_to_start = False

# Set up CSV file
timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
csv_filename = f'robot_path_{timestamp}.csv'
csv_header = ['timestamp', 'x_position', 'y_position', 'angle']

# Initialize CSV file with header
with open(csv_filename, 'w', newline='') as csvfile:
    writer = csv.writer(csvfile)
    writer.writerow(csv_header)

# Set up the real-time LiDAR scan polar plot
plt.ion()
fig, ax = plt.subplots(subplot_kw={'projection': 'polar'})
ax.set_ylim(0, 200)
ax.set_title("Current LiDAR Scan")
angles = np.linspace(0, 2 * np.pi, 24, endpoint=False)
line, = ax.plot(angles, np.zeros(24))

# Keyboard event handling
def keydown(evt):
    global dx, dy, dtheta, manual_mode
    key = evt.key
    if key == 'left': dtheta = 0.1
    elif key == 'right': dtheta = -0.1
    elif key == 'up':
        dx = env.robot_speed * np.cos(env.robot_angle)
        dy = env.robot_speed * np.sin(env.robot_angle)
    elif key == 'down':
        dx = -env.robot_speed * np.cos(env.robot_angle)
        dy = -env.robot_speed * np.sin(env.robot_angle)
    elif key == ' ': manual_mode = not manual_mode

def keyup(evt):
    global dx, dy, dtheta
    key = evt.key
    if key in ['left', 'right']: dtheta = 0
    elif key in ['up', 'down']: dx, dy = 0, 0

env.scene.bind('keydown', keydown)
env.scene.bind('keyup', keyup)

# Main loop
while True:
    rate(30)

    # Record current position to CSV
    current_time = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")
    with open(csv_filename, 'a', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow([current_time, env.robot_pos.x, env.robot_pos.y, env.robot_angle])

    # Manual control
    if manual_mode:
        env.robot_pos.x += dx
        env.robot_pos.y += dy
        env.robot_angle += dtheta
        env.robot_pos.x = np.clip(env.robot_pos.x, 50, 950)
        env.robot_pos.y = np.clip(env.robot_pos.y, 50, 950)
        returned_to_start = False
        env.current_goal = env.goal_pos
        env.status_label.text = "Manual Control"

    # Update SLAM
    slam.move_particles(dx, dy, dtheta)
    lidar_data = slam.get_lidar_scan(env.robot_pos.x, env.robot_pos.y, env.robot_angle)
    slam.update_weights(lidar_data, lidar_data)
    slam.resample_particles()
    estimated_pos = slam.estimate_position()

    # Update the LiDAR scan polar plot
    line.set_ydata(lidar_data)
    fig.canvas.draw()
    plt.pause(0.001)

    # Autonomous navigation
    if not manual_mode and not returned_to_start:
        if not path or (abs(env.robot_pos.x - path[-1][0]) < 15 and abs(env.robot_pos.y - path[-1][1]) < 15):
            path = slam.plan_path(env.robot_pos.x, env.robot_pos.y, env.current_goal.x, env.current_goal.y)
        if path:
            next_x, next_y = path[0]
            env.robot_pos.x, env.robot_pos.y, env.robot_angle = slam.move_toward_goal(env.robot_pos.x, env.robot_pos.y, env.robot_angle, next_x, next_y)
            if abs(env.robot_pos.x - next_x) < 15 and abs(env.robot_pos.y - next_y) < 15:
                path.pop(0)
            if abs(env.robot_pos.x - env.current_goal.x) < 15 and abs(env.robot_pos.y - env.current_goal.y) < 15:
                if env.current_goal == env.goal_pos:
                    env.current_goal = env.start_pos
                    env.goal_marker.pos = env.start_pos
                    env.status_label.text = "Returning to Start"
                elif env.current_goal == env.start_pos:
                    returned_to_start = True
                    env.status_label.text = "Returned to Start - Displaying SLAM Map"
        else:
            env.robot_pos.x, env.robot_pos.y, env.robot_angle = estimated_pos

    # Update map and visuals
    slam.update_map(env.robot_pos.x, env.robot_pos.y, lidar_data, env.robot_angle)
    env.update_robot_visuals()
    slam.update_particle_visuals()
    env.update_camera()
    env.mode_label.text = f"Mode: {'Manual' if manual_mode else 'Autonomous'} (Space to toggle)"

    # Exit and show map
    if returned_to_start:
        break

# Clean up and display SLAM map
plt.close(fig)
plt.ioff()
slam.show_slam_map()
print(f"Robot path data saved to {csv_filename}")
