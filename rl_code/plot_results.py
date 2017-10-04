import sys
import matplotlib.pyplot as plt
import numpy as np

# open file
f = open(sys.argv[1], "r")
contents = f.readlines()
lines = [line.rstrip('\n') for line in contents]
episode_num = []
time_step = []
wins = []
loses = []
epsilon = []

for i in lines:
	elements = i.split(',')
	episode_num.append(float(elements[0]))
	time_step.append(float(elements[1]))
	wins.append(float(elements[2]))
	loses.append(float(elements[3]))
	epsilon.append(elements[4])

print type(episode_num)
print  "yoo"

x = np.linspace(0,len(wins))
y = np.sin(x)
plt.plot(episode_num,wins,'r--',  episode_num, loses,'b--')
plt.title("Q-learning Cliff World")
plt.xlabel("Episodes")
plt.ylabel("Performance")
plt.legend(('Wins', 'Loses'), fancybox=True)
plt.show()

