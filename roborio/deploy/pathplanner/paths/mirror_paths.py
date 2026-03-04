import json
import os

FIELD_WIDTH = 8.07

def normalize_angle(angle):
    while angle > 180: angle -= 360
    while angle <= -180: angle += 360
    return angle

def mirror_y(pt):
    if pt is None: return None
    return {"x": pt["x"], "y": FIELD_WIDTH - pt["y"]}

def process_rotation(rot):
    return normalize_angle(-rot)

def mirror_path(input_path, output_path):
    with open(input_path, "r") as f:
        data = json.load(f)

    # Mirror waypoints across Y-axis
    for wp in data["waypoints"]:
        wp["anchor"] = mirror_y(wp["anchor"])
        wp["prevControl"] = mirror_y(wp["prevControl"])
        wp["nextControl"] = mirror_y(wp["nextControl"])

    # Mirror all rotation-related values
    if "rotationTargets" in data:
        for rt in data["rotationTargets"]:
            rt["rotationDegrees"] = process_rotation(rt["rotationDegrees"])

    if "goalEndState" in data and "rotation" in data["goalEndState"]:
        data["goalEndState"]["rotation"] = process_rotation(data["goalEndState"]["rotation"])

    if "idealStartingState" in data and "rotation" in data["idealStartingState"]:
        data["idealStartingState"]["rotation"] = process_rotation(data["idealStartingState"]["rotation"])

    with open(output_path, "w") as f:
        json.dump(data, f, indent=2)

if __name__ == "__main__":
    base_path = os.path.dirname(os.path.abspath(__file__))
    
    # 1. Mirror CloseSweepLeft to CloseSweepRight
    input_close = os.path.join(base_path, "CloseSweepLeft.path")
    output_close = os.path.join(base_path, "CloseSweepRight.path")
    mirror_path(input_close, output_close)
    
    # 2. Mirror FarSweepLeft to FarSweepRight
    input_far = os.path.join(base_path, "FarSweepLeft.path")
    output_far = os.path.join(base_path, "FarSweepRight.path")
    mirror_path(input_far, output_far)

    print("Successfully mirrored Left paths to Right.")
