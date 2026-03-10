from vpython import *
import numpy as np
from pathfinding.core.grid import Grid as PFGrid
from pathfinding.finder.a_star import AStarFinder
import matplotlib.pyplot as plt

class SLAM:
    def __init__(self, env):
        self.env = env
        self.GRID_SIZE = 100
        self.CELL_SIZE = 1000 // self.GRID_SIZE
        self.grid_map = np.zeros((self.GRID_SIZE, self.GRID_SIZE))

        # Particle filter
        self.NUM_PARTICLES = 50
        self.particles = np.array([[self.env.robot_pos.x, self.env.robot_pos.y, self.env.robot_angle] for _ in range(self.NUM_PARTICLES)], dtype=float)
        self.weights = np.ones(self.NUM_PARTICLES) / self.NUM_PARTICLES
        self.particle_spheres = [sphere(pos=vector(p[0], p[1], 0), radius=2, color=color.green, opacity=0.5) for p in self.particles]

        # LiDAR visualization
        self.lidar_arrows = []

    def get_lidar_scan(self, x, y, angle, num_rays=24, max_range=200):
        scan = []
        for arr in self.lidar_arrows:
            arr.visible = False
        self.lidar_arrows = []
        
        for i in range(num_rays):
            ray_angle = angle + (2 * np.pi * i / num_rays)
            for dist in range(max_range):
                ray_x = x + np.cos(ray_angle) * dist
                ray_y = y + np.sin(ray_angle) * dist
                ray_pos = vector(ray_x, ray_y, 0)
                for obj in self.env.environment_objects:
                    if isinstance(obj, box):
                        half_size = obj.size / 2
                        obj_min = obj.pos - half_size
                        obj_max = obj.pos + half_size
                        if (obj_min.x <= ray_x <= obj_max.x and 
                            obj_min.y <= ray_y <= obj_max.y and 
                            obj_min.z <= 0 <= obj_max.z):
                            scan.append(dist)
                            arrow_length = dist if dist > 0 else 1
                            self.lidar_arrows.append(arrow(pos=vector(x, y, 1), axis=vector(np.cos(ray_angle), np.sin(ray_angle), 0) * arrow_length, color=color.blue, shaftwidth=1))
                            break
                    elif isinstance(obj, (cylinder, sphere)):
                        dist_to_center = np.sqrt((ray_x - obj.pos.x)**2 + (ray_y - obj.pos.y)**2)
                        if dist_to_center <= obj.radius:
                            scan.append(dist)
                            arrow_length = dist if dist > 0 else 1
                            self.lidar_arrows.append(arrow(pos=vector(x, y, 1), axis=vector(np.cos(ray_angle), np.sin(ray_angle), 0) * arrow_length, color=color.blue, shaftwidth=1))
                            break
                else:
                    continue
                break
            else:
                scan.append(max_range)
                self.lidar_arrows.append(arrow(pos=vector(x, y, 1), axis=vector(np.cos(ray_angle), np.sin(ray_angle), 0) * max_range, color=color.blue, shaftwidth=1))
        return scan

    def get_cells_along_ray(self, start_x, start_y, end_x, end_y):
        cells = []
        x0 = int(start_x // self.CELL_SIZE)
        y0 = int(start_y // self.CELL_SIZE)
        x1 = int(end_x // self.CELL_SIZE)
        y1 = int(end_y // self.CELL_SIZE)
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy
        while True:
            if 0 <= x0 < self.GRID_SIZE and 0 <= y0 < self.GRID_SIZE:
                cells.append((x0, y0))
            if x0 == x1 and y0 == y1:
                break
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x0 += sx
            if e2 < dx:
                err += dx
                y0 += sy
        return cells

    def update_map(self, x, y, lidar_data, angle, max_range=200):
        for i, dist in enumerate(lidar_data):
            ray_angle = angle + (2 * np.pi * i / len(lidar_data))
            end_x = x + np.cos(ray_angle) * dist
            end_y = y + np.sin(ray_angle) * dist
            cells = self.get_cells_along_ray(x, y, end_x, end_y)
            if dist < max_range:
                for cell in cells[:-1]:
                    self.grid_map[cell[1], cell[0]] = 0  # Free space
                if cells:
                    self.grid_map[cells[-1][1], cells[-1][0]] = 1  # Occupied
            else:
                for cell in cells:
                    self.grid_map[cell[1], cell[0]] = 0  # Free space

    def move_particles(self, dx, dy, dtheta, noise=0.2):
        self.particles[:, 0] += dx + np.random.normal(0, noise, self.NUM_PARTICLES)
        self.particles[:, 1] += dy + np.random.normal(0, noise, self.NUM_PARTICLES)
        self.particles[:, 2] += dtheta + np.random.normal(0, noise, self.NUM_PARTICLES)
        self.particles[:, 0] = np.clip(self.particles[:, 0], 50, 950)
        self.particles[:, 1] = np.clip(self.particles[:, 1], 50, 950)
        return self.particles

    def update_weights(self, lidar_data, real_lidar):
        for i, particle in enumerate(self.particles):
            px, py, ptheta = particle
            particle_scan = self.get_lidar_scan(px, py, ptheta, num_rays=len(lidar_data))
            error = sum((d - r) ** 2 for d, r in zip(particle_scan, real_lidar))
            self.weights[i] = 1 / (error + 1e-6)
        weight_sum = self.weights.sum()
        if weight_sum > 0:
            self.weights /= weight_sum
        else:
            self.weights[:] = 1 / self.NUM_PARTICLES
        return self.weights

    def resample_particles(self):
        if not np.isfinite(self.weights).all() or self.weights.sum() == 0:
            self.weights[:] = 1 / self.NUM_PARTICLES
        else:
            self.weights /= self.weights.sum()
        indices = np.random.choice(self.NUM_PARTICLES, size=self.NUM_PARTICLES, p=self.weights)
        self.particles = self.particles[indices]
        return self.particles

    def estimate_position(self):
        return np.average(self.particles, weights=self.weights, axis=0)

    def plan_path(self, start_x, start_y, goal_x, goal_y):
        pf_matrix = (1 - self.grid_map).tolist()
        pf_grid = PFGrid(matrix=pf_matrix)
        start = pf_grid.node(int(start_x // self.CELL_SIZE), int(start_y // self.CELL_SIZE))
        end = pf_grid.node(int(goal_x // self.CELL_SIZE), int(goal_y // self.CELL_SIZE))
        finder = AStarFinder()
        path, _ = finder.find_path(start, end, pf_grid)
        return [(p.x * self.CELL_SIZE, p.y * self.CELL_SIZE) for p in path]

    def move_toward_goal(self, robot_x, robot_y, robot_angle, goal_x, goal_y):
        angle_to_goal = np.arctan2(goal_y - robot_y, goal_x - robot_x)
        angle_diff = (angle_to_goal - robot_angle + np.pi) % (2 * np.pi) - np.pi
        if abs(angle_diff) > 0.05:
            robot_angle += 0.1 * np.sign(angle_diff)
        else:
            robot_x += self.env.robot_speed * np.cos(robot_angle)
            robot_y += self.env.robot_speed * np.sin(robot_angle)
        return robot_x, robot_y, robot_angle

    def update_particle_visuals(self):
        for i, p in enumerate(self.particles):
            self.particle_spheres[i].pos = vector(p[0], p[1], 0)

    def show_slam_map(self):
        plt.imshow(self.grid_map, cmap='gray', origin='lower')
        plt.title("Final SLAM Map")
        plt.xlabel("X (meters)")
        plt.ylabel("Y (meters)")
        plt.show()