import sys
import rosbag
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import pyplot
import numpy as np



time_step = []
episode = []
pitch = []
actions = []
random = []


pitch_t = np.array([])
pitch_a = np.array([])

bag = rosbag.Bag(sys.argv[1])
for (topic, msg, t) in bag.read_messages():
	if msg.msg[0:2] == 'Ts':
		time_step.append(msg.msg[3:])
	elif msg.msg[0:2] == 'Ep':
		episode.append(msg.msg[3:])
	elif msg.msg[0:2] == 'Pa':
		pitch.append(msg.msg[3:])
#		np.append(pitch_t,t)		
#		np.append(pitch_a,msg.msg[3:])
	elif msg.msg[0] == 'A':
		actions.append(msg.msg[2:])
	else:
		random.append(msg.msg)



fig = pyplot.figure()
ax = Axes3D(fig)

#fill in actions
a_cntr = 0
ep_cntr = 0
actions_filled = []
for i,t in enumerate(time_step):
	actions_filled.append(actions[a_cntr])
	if t == '0' and i > 0:
		ep_cntr = ep_cntr + 1
		if ep_cntr % 2 == 0:
			a_cntr = a_cntr + 1


time_step_int = [int(i) for i in time_step]
pitch_float = [float(i) for i in pitch]
actions_filled_int = [int(i) for i in actions_filled]


ax.scatter(time_step_int, actions_filled_int, pitch_float)

ax.set_xlabel("Time steps")
ax.set_ylabel("actions")
ax.set_zlabel("pitch angle")


pyplot.show()

fig = pyplot.figure()
ax = Axes3D(fig)
ax.plot_surface(actions_filled_int, pitch_float,time_step_int, color='b')
pyplot.show()

fig = pyplot.figure()
ax = Axes3D(fig)
ax.plot_wireframe(time_step_int, actions_filled_int, pitch_float,rstride=10, cstride=10)# color='b')
pyplot.show()


