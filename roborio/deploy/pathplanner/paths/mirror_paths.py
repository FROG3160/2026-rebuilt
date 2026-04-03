import json
import os

FIELD_WIDTH = 8.07


def normalize_angle(angle):
    while angle > 180:
        angle -= 360
    while angle <= -180:
        angle += 360
    return angle


def mirror_y(pt):
    if pt is None:
        return None
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
        data["goalEndState"]["rotation"] = process_rotation(
            data["goalEndState"]["rotation"]
        )

    if "idealStartingState" in data and "rotation" in data["idealStartingState"]:
        data["idealStartingState"]["rotation"] = process_rotation(
            data["idealStartingState"]["rotation"]
        )

    with open(output_path, "w") as f:
        json.dump(data, f, indent=2)


if __name__ == "__main__":
    import sys
    base_path = os.path.dirname(os.path.abspath(__file__))

    if len(sys.argv) > 1:
        path_name = sys.argv[1]
        input_path = os.path.join(base_path, path_name)
        if not os.path.exists(input_path):
            input_path = path_name
        output_path = input_path.replace("Left", "Right")
        print(f"Mirroring {input_path} to {output_path}")
        mirror_path(input_path, output_path)
        print("Successfully mirrored specified path.")
    else:
        # Find all paths in the directory that contain "Left", but not the words "Tower", "Outpost", or "Corral
        # " in their name
        def check_name(name):
            return (
                "Tower" not in name
                and "Outpost" not in name
                and "Corral" not in name
                and "Left" in name
            )

        left_paths_to_mirror = [
            f for f in os.listdir(base_path) if check_name(f) and f.endswith(".path")
        ]
        # ]

        for path in left_paths_to_mirror:
            input_path = os.path.join(base_path, path)
            output_path = os.path.join(base_path, path.replace("Left", "Right"))
            print(f"Mirroring {input_path} to {output_path}")
            mirror_path(input_path, output_path)

        print("Successfully mirrored Left paths to Right.")

    # # 1. Mirror CloseSweepLeft to CloseSweepRight
    # input_close = os.path.join(base_path, "NearSweepLeft.path")
    # output_close = os.path.join(base_path, "NearSweepRight.path")
    # mirror_path(input_close, output_close)

    # # 2. Mirror FarSweepLeft to FarSweepRight
    # input_far = os.path.join(base_path, "FarSweepLeft.path")
    # output_far = os.path.join(base_path, "FarSweepRight.path")
    # mirror_path(input_far, output_far)

    # input_loop = os.path.join(base_path, "LeftSideLoop.path")
    # output_loop = os.path.join(base_path, "RightSideLoop.path")

    # print("Successfully mirrored Left paths to Right.")
