import pose
import matplotlib.pyplot as plt

pose_recorder = MPU9250()
pos_x = []
pos_y = []
position = [0, 0]
velocity = [0, 0]
acceleration = [0, 0]

try:
    while True:
		acceleration = pose_recorder.read_acceleration()
		velocity[0] = velocity[0] + acceleration[0]
		velocity[1] = velocity[1] + acceleration[1]
		position[0] = position[0] + velocity[0]
		position[1] = position[1] + velocity[1]
		pos_x.append(position[0])
		pos_y.append(position[1])
except KeyboardInterrput:
	pass
finally:
    plt.scatter(pos_x, pos_y)
