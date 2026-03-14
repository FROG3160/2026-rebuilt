import math

# Gearbox reduction
gear_reduction = 5.0

# Sprocket reduction
driving_sprocket = 19.0
driven_sprocket = 14.0
sprocket_ratio = driven_sprocket / driving_sprocket

# Wheel properties
wheel_diameter_inches = 4.0
wheel_circumference_inches = wheel_diameter_inches * math.pi
wheel_circumference_meters = wheel_circumference_inches * 0.0254

# Total reduction from motor to wheel (motor revolutions per wheel revolution)
total_mechanical_ratio = gear_reduction * (driven_sprocket / driving_sprocket)

# The sensor_to_mechanism_ratio in CTRE config is the ratio from motor rotations to mechanism units.
# We want the mechanism units to be linear distance (meters).
# Therefore, sensor_to_mechanism_ratio = (motor rotations per wheel rotation) / (wheel circumference in meters)
sensor_to_mechanism_ratio = total_mechanical_ratio / wheel_circumference_meters

print(f"Gear Reduction: {gear_reduction}")
print(f"Sprocket Ratio (Driven/Driving): {driven_sprocket}/{driving_sprocket} = {sprocket_ratio}")
print(f"Total Mechanical Ratio: {total_mechanical_ratio}")
print(f"Wheel Circumference (m): {wheel_circumference_meters}")
print(f"Calculated sensor_to_mechanism_ratio: {sensor_to_mechanism_ratio}")
