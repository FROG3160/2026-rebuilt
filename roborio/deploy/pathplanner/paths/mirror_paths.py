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

def mirror_path(input_path, output_path, mirror_axis, start_rot_override=None, end_x_override=None, end_rot_override=None):
    with open(input_path, "r") as f:
        data = json.load(f)

    # Mirror waypoints
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

    # Basic mirroring for rotations
    if "rotationTargets" in data:
        for rt in data["rotationTargets"]:
            rt["rotationDegrees"] = process_rotation(rt["rotationDegrees"], mirror_axis)

    if "goalEndState" in data and "rotation" in data["goalEndState"]:
        data["goalEndState"]["rotation"] = process_rotation(data["goalEndState"]["rotation"], mirror_axis)

    if "idealStartingState" in data and "rotation" in data["idealStartingState"]:
        data["idealStartingState"]["rotation"] = process_rotation(data["idealStartingState"]["rotation"], mirror_axis)

    # Apply Overrides
    if start_rot_override is not None:
        if "idealStartingState" in data:
            data["idealStartingState"]["rotation"] = start_rot_override
        if "rotationTargets" in data and len(data["rotationTargets"]) > 0:
            # Assuming first rotation target is at pos 0
            if data["rotationTargets"][0]["waypointRelativePos"] == 0:
                data["rotationTargets"][0]["rotationDegrees"] = start_rot_override

    if end_x_override is not None:
        # Move the last waypoint anchor to the new X
        # For simplicity, we also move the previous control point by the same delta
        last_wp = data["waypoints"][-1]
        delta_x = end_x_override - last_wp["anchor"]["x"]
        last_wp["anchor"]["x"] = end_x_override
        if last_wp["prevControl"] is not None:
             last_wp["prevControl"]["x"] += delta_x

    if end_rot_override is not None:
        if "goalEndState" in data:
            data["goalEndState"]["rotation"] = end_rot_override
        if "rotationTargets" in data and len(data["rotationTargets"]) > 1:
            # Assuming last rotation target is the one we want to override
             data["rotationTargets"][-1]["rotationDegrees"] = end_rot_override

    with open(output_path, "w") as f:
        json.dump(data, f, indent=2)

if __name__ == "__main__":
    base_path = os.path.dirname(os.path.abspath(__file__))
    input_file = os.path.join(base_path, "CloseSweepLeft.path")
    
    # Get CloseSweepLeft values to use as overrides
    with open(input_file, "r") as f:
        cs_left = json.load(f)
    
    close_end_x = cs_left["waypoints"][-1]["anchor"]["x"]
    close_end_rot = cs_left["goalEndState"]["rotation"]

    # CloseSweepRight (Just mirror Y)
    mirror_path(input_file, os.path.join(base_path, "CloseSweepRight.path"), "y")
    
    # FarSweepLeft (Mirror X, but override end point and start rotation)
    mirror_path(input_file, os.path.join(base_path, "FarSweepLeft.path"), "x", 
                start_rot_override=110.0, 
                end_x_override=close_end_x, 
                end_rot_override=close_end_rot)
    
    # FarSweepRight (Mirror XY, but override end point and start rotation)
    mirror_path(input_file, os.path.join(base_path, "FarSweepRight.path"), "xy", 
                start_rot_override=-110.0, 
                end_x_override=close_end_x, 
                end_rot_override=-close_end_rot)

    print("Generated mirrored paths with Far-zone overrides successfully.")
