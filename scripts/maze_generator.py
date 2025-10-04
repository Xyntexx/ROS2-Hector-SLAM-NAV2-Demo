#!/usr/bin/env python3
import random
from pathlib import Path

# Maze generation using recursive backtracking
def generate_maze(width, height):
    maze = [[1 for _ in range(width)] for _ in range(height)]

    def carve(x, y):
        dirs = [(2, 0), (-2, 0), (0, 2), (0, -2)]
        random.shuffle(dirs)
        for dx, dy in dirs:
            nx, ny = x + dx, y + dy
            if 0 < nx < width and 0 < ny < height and maze[ny][nx] == 1:
                maze[y + dy // 2][x + dx // 2] = 0
                maze[ny][nx] = 0
                carve(nx, ny)

    # start point
    maze[1][1] = 0
    carve(1, 1)
    return maze

def sdf_cylinder_model(name, x, y, z, radius, height, color):
    return f"""
    <model name='{name}'>
      <static>true</static>
      <pose>{x} {y} {z/2} 0 0 0</pose>
      <link name='link'>
        <collision name='collision'>
          <geometry>
            <cylinder>
              <radius>{radius}</radius>
              <length>{height}</length>
            </cylinder>
          </geometry>
        </collision>
        <visual name='visual'>
          <geometry>
            <cylinder>
              <radius>{radius}</radius>
              <length>{height}</length>
            </cylinder>
          </geometry>
          <material>
            <ambient>{color}</ambient>
            <diffuse>{color}</diffuse>
          </material>
        </visual>
      </link>
    </model>
    """

def maze_to_sdf(maze, cell_size=1.0, wall_thickness=0.1, wall_height=1.0):
    world = """<sdf version='1.8'>
  <world name='block_maze'>
    <!-- Physics -->
    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <!-- Lighting -->
    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <attenuation>
        <range>1000</range>
        <constant>0.9</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

    <!-- Ground plane -->
    <model name='ground_plane'>
      <static>true</static>
      <link name='link'>
        <collision name='collision'>
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
        </collision>
        <visual name='visual'>
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <material>
            <ambient>0.5 0.5 0.5 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

"""
    rows, cols = len(maze), len(maze[0])
    for y in range(rows):
        for x in range(cols):
            if maze[y][x] == 1:
                wx = (x - cols/2) * cell_size
                wy = (y - rows/2) * cell_size
                # Create cylindrical pillars (rounded appearance)
                world += sdf_cylinder_model(
                    name=f"wall_{x}_{y}",
                    x=wx, y=wy, z=wall_height,
                    radius=cell_size/2,  # Cylinder fits in cell
                    height=wall_height*2,
                    color="0.6 0.6 0.6 1"  # Medium gray
                )
    world += "\n  </world>\n</sdf>\n"
    return world


def main():
    width, height = 15, 15   # odd numbers work best
    cell_size = 1.0
    wall_thickness = 0.1
    wall_height = 0.2
    output_file = Path("block_maze.world")

    maze = generate_maze(width, height)
    sdf_content = maze_to_sdf(maze, cell_size, wall_thickness, wall_height)
    output_file.write_text(sdf_content)
    print(f"✅ Maze world generated: {output_file.resolve()}")

if __name__ == "__main__":
    main()
