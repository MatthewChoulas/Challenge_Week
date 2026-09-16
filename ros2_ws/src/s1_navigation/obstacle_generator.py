import math
import random
from pathlib import Path
import json

# Global Constants, edit to change obstacle generation parameters

NUM_OBSTACLES = 30

MIN_RADIUS = 0.5
MAX_RADIUS = 0.8

X_MIN = -10.0
X_MAX = 10.0
Y_MIN = -10.0
Y_MAX = 10.0

ROBOT_X = 0.0
ROBOT_Y = 0.0

ROBOT_CLEARANCE = 0.75

RANDOM_SEED = None

TEMPLATE_FILE = "worlds/s1_world_template.sdf"
OUTPUT_FILE = "worlds/s1_world.sdf"
OBSTACLE_FILE = "worlds/obstacles.json"

OBSTACLE_MARKER = "<!-- RANDOM OBSTACLES -->"


# return configuration to generate a sphere
def make_sphere_model(index, x, y, radius):

    return f"""
    <model name="rock_{index}">
      <static>true</static>
      <pose>{x:.4f} {y:.4f} 0 0 0 0</pose>
      <link name="link">

        <collision name="collision">
          <geometry>
            <sphere>
              <radius>{radius:.4f}</radius>
            </sphere>
          </geometry>
        </collision>

        <visual name="visual">
          <geometry>
            <sphere>
              <radius>{radius:.4f}</radius>
            </sphere>
          </geometry>

          <material>
            <ambient>0.30 0.22 0.15 1</ambient>
            <diffuse>0.35 0.27 0.18 1</diffuse>
          </material>
        </visual>

      </link>
    </model>
"""


# check if obstacle clips robot at its starting position
def clips_robot(x, y, radius):
    distance = math.hypot(x - ROBOT_X, y - ROBOT_Y)
    minimum_distance = radius + ROBOT_CLEARANCE
    return distance < minimum_distance


# generate obstacles randomly
def generate_obstacles():
    rng = random.Random(RANDOM_SEED)

    obstacles = []
    obstacle_models = []
    for i in range(NUM_OBSTACLES):
        radius = rng.uniform(MIN_RADIUS, MAX_RADIUS)

        while True:
            x = rng.uniform(X_MIN + radius, X_MAX - radius)
            y = rng.uniform(Y_MIN + radius,Y_MAX - radius)

            # reject samples that clip the robot
            if clips_robot(x, y, radius):
                continue

            break

        obstacle_models.append(make_sphere_model(i, x, y, radius))
        obstacles.append({"x": x, "y": y, "radius": radius})


        print(f"Obstacle {i}: "
            f"x={x:.2f}, "
            f"y={y:.2f}, "
            f"r={radius:.2f}"
        )

    with open(OBSTACLE_FILE, "w") as f:
        json.dump(obstacles, f, indent=4)

    return "\n".join(obstacle_models)


def main():

    template_path = Path(TEMPLATE_FILE)
    output_path = Path(OUTPUT_FILE)

    if not template_path.exists():
        raise FileNotFoundError(
            f"Template world not found: {template_path}"
        )

    world = template_path.read_text()

    if OBSTACLE_MARKER not in world:
        raise RuntimeError(
            f"Could not find marker "
            f"{OBSTACLE_MARKER} "
            f"in {template_path}"
        )

    obstacles = generate_obstacles()

    world = world.replace(OBSTACLE_MARKER, obstacles)
    output_path.write_text(world)

    print(f"Generated {NUM_OBSTACLES} obstacles.")



if __name__ == "__main__":
    main()