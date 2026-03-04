import json
import os

FIELD_WIDTH = 8.07
FIELD_LENGTH = 16.54

def normalize_angle(angle):
    while angle > 180: angle -= 360
    while angle <= -180: angle += 360
    return angle

def mirror_y(pt):
    if pt is None: return None
    return {"x": pt["x"], "y": FIELD_WIDTH - pt["y"]}

def mirror_x(pt):
    if pt is None: return None
    return {"x": FIELD_LENGTH - pt["x"], "y": pt["y"]}

def mirror_xy(pt):
    if pt is None: return None
    return {"x": FIELD_LENGTH - pt["x"], "y": FIELD_WIDTH - pt["y"]}

def process_rotation(rot, mirror_axis):
    if mirror_axis == 'y': return normalize_angle(-rot)
    elif mirror_axis == 'x': return normalize_angle(180 - rot)
    elif mirror_axis == 'xy': return normalize_angle(180 + rot)

def mirror_path(input_path, output_path, mirror_axis):
    with open(input_path, "r") as f:
        data = json.load(f)

    for wp in data["waypoints"]:
        if mirror_axis == 'y':
            wp["anchor"] = mirror_y(wp["anchor"])
            wp["prevControl"] = mirror_y(wp["prevControl"])
            wp["nextControl"] = mirror_y(wp["nextControl"])
        elif mirror_axis == 'x':
            wp["anchor"] = mirror_x(wp["anchor"])
            wp["prevControl"] = mirror_x(wp["prevControl"])
            wp["nextControl"] = mirror_x(wp["nextControl"])
        elif mirror_axis == 'xy':
            wp["anchor"] = mirror_xy(wp["anchor"])
            wp["prevControl"] = mirror_xy(wp["prevControl"])
            wp["nextControl"] = mirror_xy(wp["nextControl"])

    if "rotationTargets" in data:
        for rt in data["rotationTargets"]:
            rt["rotationDegrees"] = process_rotation(rt["rotationDegrees"], mirror_axis)

    if "goalEndState" in data and "rotation" in data["goalEndState"]:
        data["goalEndState"]["rotation"] = process_rotation(data["goalEndState"]["rotation"], mirror_axis)

    if "idealStartingState" in data and "rotation" in data["idealStartingState"]:
        data["idealStartingState"]["rotation"] = process_rotation(data["idealStartingState"]["rotation"], mirror_axis)

    with open(output_path, "w") as f:
        json.dump(data, f, indent=2)

if __name__ == "__main__":
    base_path = os.path.dirname(os.path.abspath(__file__))
    input_file = os.path.join(base_path, "CloseSweepLeft.path")

    mirror_path(input_file, os.path.join(base_path, "CloseSweepRight.path"), "y")
    mirror_path(input_file, os.path.join(base_path, "FarSweepLeft.path"), "x")
    mirror_path(input_file, os.path.join(base_path, "FarSweepRight.path"), "xy")

    print("Generated mirrored paths successfully.")
