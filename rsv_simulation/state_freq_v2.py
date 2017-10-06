'''
Code plots the frequnecy of states visited
'''


import sys
import rosbag
import matplotlib.pyplot as plt; plt.rcdefaults()
import matplotlib.pyplot as plt
import numpy as np
#import plot_curve


time_step = []
episode = []
pitch = []
actions = []
random = []


pitch_t = np.array([])
pitch_a = np.array([])

state = []
bag = rosbag.Bag(sys.argv[1])
for (topic, msg, t) in bag.read_messages():
        if topic == '/rosout':
            if msg.msg[0:5] == 'state':
		state.append(int(msg.msg[7:]))


# count every time a state appears
zeros = lambda x : [0]*x
state_freq = zeros(120)

for state_cntr, i in enumerate(state):
    state_freq[i] = state_freq[i] + 1

print len(state_freq)
all_states = range(0, len(state_freq), 1)
filtered_all_states = [x for x in all_states if x % 10 == 0]

fig = plt.figure()
ax = fig.add_subplot(111)

plot = ax.bar(all_states, state_freq, align='center')
#plt.xticks(all_states)
ax.set_xticks(filtered_all_states)
ax.set_xticklabels(filtered_all_states, rotation=45)
plt.xlabel('States')
plt.ylabel('Frequnecy')
plt.title('State vist frequnecy')
plt.show()



