import argparse
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

PACKAGE_DIR = Path(__file__).resolve().parent
TEMPLATE_FILE = PACKAGE_DIR / "worlds/s1_world_template.sdf"
OUTPUT_FILE = PACKAGE_DIR / "worlds/s1_world.sdf"
OBSTACLE_FILE = PACKAGE_DIR / "worlds/obstacles.json"
MAX_SAMPLE_ATTEMPTS = 10000

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
def generate_obstacles(obstacle_file=None):
    rng = random.Random(RANDOM_SEED)

    obstacles = []
    obstacle_models = []
    for i in range(NUM_OBSTACLES):
        radius = round(rng.uniform(MIN_RADIUS, MAX_RADIUS), 4)

        for _ in range(MAX_SAMPLE_ATTEMPTS):
            x = round(rng.uniform(X_MIN + radius, X_MAX - radius), 4)
            y = round(rng.uniform(Y_MIN + radius, Y_MAX - radius), 4)

            # reject samples that clip the robot
            if clips_robot(x, y, radius):
                continue

            break
        else:
            raise RuntimeError("Unable to place obstacle with the configured clearance")

        obstacle_models.append(make_sphere_model(i, x, y, radius))
        obstacles.append({"x": x, "y": y, "radius": radius})


        print(f"Obstacle {i}: "
            f"x={x:.2f}, "
            f"y={y:.2f}, "
            f"r={radius:.2f}"
        )

    with open(obstacle_file or OBSTACLE_FILE, "w") as f:
        json.dump(obstacles, f, indent=4)

    return "\n".join(obstacle_models)


def main(args=None):
    parser = argparse.ArgumentParser(description="Generate a world and matching obstacle metadata")
    parser.add_argument("--template", type=Path, default=TEMPLATE_FILE)
    parser.add_argument("--output-dir", type=Path, default=Path(OUTPUT_FILE).parent)
    options = parser.parse_args(args)

    template_path = options.template
    output_path = options.output_dir / "s1_world.sdf"

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

    options.output_dir.mkdir(parents=True, exist_ok=True)
    obstacles = generate_obstacles(options.output_dir / "obstacles.json")

    world = world.replace(OBSTACLE_MARKER, obstacles)
    output_path.write_text(world)

    print(f"Generated {NUM_OBSTACLES} obstacles.")



if __name__ == "__main__":
    main()