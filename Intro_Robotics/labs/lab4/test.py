
lower_bound = (0.75/20) * 100
upper_bound = (2.25/20) * 100
angle = 0
angle_percentage = (angle + 90.0) / 180.0
pulse_width = (1 - angle_percentage) * lower_bound + angle_percentage * upper_bound;
print(pulse_width)
print(lower_bound)
print(upper_bound)