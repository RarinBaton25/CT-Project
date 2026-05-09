def clockwise_or_anticlockwise(angle1, angle2):
    clockwise_difference        = max(angle1, angle2)     - min(angle1, angle2)
    anticlockwise_difference    = max(angle1, angle2)-360 - min(angle1, angle2)

    if abs(clockwise_difference) <= abs(anticlockwise_difference):
        return 1
    else:
        return -1
    

for angles in [
    [2  , 4],
    [54 , 267],
    [100, 101],

    [358, 4],
    [270, 100],
    [0  , 180]
]:
    if clockwise_or_anticlockwise(angles[0], angles[1]) == 1:
        print(angles, "Clockwise")
    else:
        print(angles, "Anti-clockwise")
