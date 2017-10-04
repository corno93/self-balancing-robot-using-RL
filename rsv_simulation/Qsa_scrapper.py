'''
Code gets the final Q(s,a) matrix and stores it in a text file
'''

import re
import sys
import rosbag
import matplotlib.pyplot as plt; plt.rcdefaults()
import matplotlib.pyplot as plt
import numpy as np
import os
#import plot_curve

def get_single_elements(item):
    if hasattr(item, '__iter__'):
        for child_item in item:
            for element in get_single_elements(child_item):
                yield element
    else:
        yield item

def print_flat(item):
    print [element for element in get_single_elements(item)]



msg_list = []

bag = rosbag.Bag(sys.argv[1])
for (topic, msg, t) in bag.read_messages():
        if topic == '/Q_state':
		msg_list.append( str(msg))

qsa = msg_list[-1]

#re_data = re.split(':', qsa)
#print re_data[0:2]
#sys.exit()

filtered_x3 = []
filtered_qsa = qsa.translate(None, '[]state:\n')
filtered = [x.strip() for x in filtered_qsa.split(',')]
filtered_x2 = [x.split() for x in filtered]
for i in filtered_x2:
	if len(i) == 2:
		filtered_x3.append(i[0])
		filtered_x3.append(i[1])
	else:
		filtered_x3.append(i)


filtered_x3.pop(0)
filtered_final = [x for x in get_single_elements(filtered_x3)]
print filtered_final[0:28]

#sys.exit()
log_file = os.getcwd() + '/Qsa_' + sys.argv[1][3:-4]+ '.txt'
print log_file
try:
	file = open(log_file, 'w')
except:
	print "file %s already exists" % log_file
cntr = 1
for j in filtered_final:
	file.write(j)
	if (cntr % 7 == 0):
		file.write("\n")
	else:
		file.write(",")
	cntr = cntr + 1


sys.exit()
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



