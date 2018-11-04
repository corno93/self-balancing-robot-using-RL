import sys
import rosbag
import matplotlib.pyplot as plt
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


print time_step[:70]
print pitch[:70]
print pitch_t
print pitch_a


#pitch_plot = np.column_stack(pitch_t, pitch_a)
#plt.plot(pitch_plot, 'bo')
#plt.show()

plt.plot(time_step, pitch, 'ro')	
plt.show()

#time_data = np.arange(0, len(pitch_data[3:])/2, 0.5)

#print len(pitch_data)
#print len(time_data)


#plt.plot(time_data, pitch_data[3:])
#plt.ylabel('Pitch') 
#plt.xlabel('Time')
#plt.show()

