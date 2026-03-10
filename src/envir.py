from vpython import *
import numpy as np

class RobotEnvironment:
    def __init__(self):
        # Scene setup
        self.scene = canvas(width=1000, height=800, center=vector(500, 500, 0), background=color.white)
        self.scene.forward = vector(0, -0.3, -1)  # Slightly tilted top-down view
        distant_light(direction=vector(0, -1, -1), color=color.white)  # Add lighting

        # Robot properties
        self.robot_pos = vector(500, 500, 0)  # Starting position at ground level
        self.robot_angle = 0
        self.robot_speed = 15
        self.CAR_LENGTH = 30
        self.CAR_WIDTH = 15
        self.CAR_HEIGHT = 10
        self.WHEEL_RADIUS = 3

        # Road environment
        road_width = 900
        road_height = 900
        road_thickness = 2
        self.road = box(pos=vector(500, 500, -road_thickness/2), size=vector(road_width, road_height, road_thickness), color=color.gray(0.7))

        # Environment objects with additional obstacles
        self.environment_objects = [
            # Boundaries
            box(pos=vector(50, 500, 10), size=vector(10, road_height, 20), color=color.black),
            box(pos=vector(950, 500, 10), size=vector(10, road_height, 20), color=color.black),
            box(pos=vector(500, 50, 10), size=vector(road_width, 10, 20), color=color.black),
            box(pos=vector(500, 950, 10), size=vector(road_width, 10, 20), color=color.black),
            # Original obstacles
            box(pos=vector(300, 200, 5), size=vector(200, 10, 10), color=color.yellow),
            box(pos=vector(200, 300, 5), size=vector(10, 200, 10), color=color.yellow),
            box(pos=vector(700, 800, 5), size=vector(200, 10, 10), color=color.yellow),
            box(pos=vector(800, 700, 5), size=vector(10, 200, 10), color=color.yellow),
            box(pos=vector(500, 500, 0), size=vector(100, 10, 1), color=color.white),
            box(pos=vector(500, 500, 0), size=vector(10, 100, 1), color=color.white),
            cylinder(pos=vector(150, 150, 0), radius=10, height=30, color=color.green),
            box(pos=vector(850, 150, 15), size=vector(50, 50, 30), color=color.red),
            # New obstacles
            box(pos=vector(400, 600, 5), size=vector(10, 200, 10), color=color.blue),
            box(pos=vector(600, 400, 5), size=vector(200, 10, 10), color=color.blue),
            box(pos=vector(300, 800, 5), size=vector(10, 100, 10), color=color.orange),
            box(pos=vector(700, 300, 5), size=vector(100, 10, 10), color=color.orange),
            cylinder(pos=vector(500, 700, 0), radius=20, height=20, color=color.purple),
            sphere(pos=vector(200, 800, 10), radius=15, color=color.magenta),
            sphere(pos=vector(400, 400, 10), radius=20, color=color.cyan),
            box(pos=vector(600, 600, 5), size=vector(50, 50, 10), color=color.green),

            box(pos=vector(400, 600, 5), size=vector(50, 50, 10), color=color.blue),
            box(pos=vector(600, 400, 5), size=vector(50, 50, 10), color=color.blue),
            box(pos=vector(800, 200, 5), size=vector(10, 100, 10), color=color.orange),
            box(pos=vector(200, 800, 5), size=vector(100, 10, 10), color=color.orange),

            cylinder(pos=vector(750, 250, 0), radius=15, height=20, color=color.green),
            sphere(pos=vector(250, 750, 10), radius=10, color=color.magenta),
            #box(pos=vector(700, 500, 5), size=vector(20, 20, 10), color=color.cyan),
        ]

        # Goals
        self.start_pos = vector(500, 500, 0)
        self.goal_pos = vector(850, 850, 0)
        self.current_goal = self.goal_pos
        self.goal_marker = sphere(pos=self.goal_pos, radius=5, color=color.red)

        # Robot model
        self.car_body = box(pos=vector(self.robot_pos.x, self.robot_pos.y, self.CAR_HEIGHT/2 + self.WHEEL_RADIUS), 
                            size=vector(self.CAR_LENGTH, self.CAR_WIDTH, self.CAR_HEIGHT), 
                            color=color.red, 
                            axis=vector(self.CAR_LENGTH, 0, 0))
        self.wheel_offsets = [
            vector(self.CAR_LENGTH/4, -self.CAR_WIDTH/2, 0),  # front-left
            vector(self.CAR_LENGTH/4, self.CAR_WIDTH/2, 0),   # front-right
            vector(-self.CAR_LENGTH/4, -self.CAR_WIDTH/2, 0), # rear-left
            vector(-self.CAR_LENGTH/4, self.CAR_WIDTH/2, 0)   # rear-right
        ]
        self.wheels = [
            cylinder(pos=vector(0, 0, self.WHEEL_RADIUS), radius=self.WHEEL_RADIUS, length=3, color=color.black, axis=vector(0, 1, 0)) 
            for _ in range(4)
        ]

        # Labels
        self.mode_label = label(pos=vector(500, 980, 0), text="Mode: Manual (Space to toggle)", box=False)
        self.status_label = label(pos=vector(500, 950, 0), text="Heading to Goal", box=False)

    def update_robot_visuals(self):
        self.car_body.pos = vector(self.robot_pos.x, self.robot_pos.y, self.CAR_HEIGHT/2 + self.WHEEL_RADIUS)
        self.car_body.axis = vector(np.cos(self.robot_angle), np.sin(self.robot_angle), 0) * self.CAR_LENGTH
        for i, wheel in enumerate(self.wheels):
            rotated_offset = self.wheel_offsets[i].rotate(angle=self.robot_angle, axis=vector(0, 0, 1))
            wheel.pos = vector(self.robot_pos.x + rotated_offset.x, self.robot_pos.y + rotated_offset.y, self.WHEEL_RADIUS)
            wheel.axis = vector(-np.sin(self.robot_angle), np.cos(self.robot_angle), 0) * 3

    def update_camera(self):
        self.scene.center = vector(self.robot_pos.x, self.robot_pos.y, 0)
        self.scene.range = 300  # Zoom level