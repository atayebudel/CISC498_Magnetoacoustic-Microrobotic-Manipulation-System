# Initialize buffers for 6 sensors, each with 40 points (start at 0)
buffer_size = 40
num_sensors = 6
buffers = [[0]*buffer_size for _ in range(num_sensors)]

# Suppose this is the new reading from Arduino:
new_data = [0.1, 0.2, 0.0, -0.1, 0.05, -0.05]  # example currents

# Shift old values and add new value for each sensor
for i in range(num_sensors):
    buffers[i] = buffers[i][1:] + [new_data[i]]