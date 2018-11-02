# self-balancing-robot-using-RL

This repository contains all the source code used throughout my under-graduate thesis for the degree of a Bachelor of Mechatronic engineering at The University of Sydney. The thesis was titled 'Balancing a Two Wheeled Robot with Reinforcement Learning' that has an aim of creating an insight for using Reinforcement Learning (RL) algorithms in the field of robotic actuated control systems. This thesis consisted of the design and development of a two-wheeled robot using ROS as its software archieture as well as three experiments:

- **Grid world**:
Here RL algorithms Q-Learning and SARSA (state-action-reward-state-action) are tested in a grid world environment. The grid world environment simply consists of a grid n by n in size, an agent, a goal state and a reward for each state transition. The agent uses the RL algorithm to build an optimal value function whereby it will reach the goal state as quickly as possible. The reason for this experiment was to write these algorithms in C++ in a simple environment and to develop a thorough understanding of them before implementing them in more complex scenarious. 
Please see the directory rl_code. There are installation and build instructions. 

- **Gazebo simulation**:
Here the RL algorithms are compared against the traditional PID algorithm in achieving balance for a two-wheeled robot in simulation of a two-wheeled robot. Courtesy of [RoboSavvy](http://wiki.ros.org/Robots/RoboSavvy-Balance), this open-sourced simulation uses the ROS interface in a Gazebo simulation of a two-wheeled robot. Comparisons are made between each algorithm by testing their preromance for the following:
    - Terrain test. The simulation provides a flat and a rocky terrain environment.
    - Robustness test. A large disturbance force is inputted onto the chassis.
    - Adaptability test. The simulated robot will have a significant change in its mass and weigth distribution. 
Please see the directory XXX.

- **Robot**:
Now the algorithms are finally tested on the built robot. 
Please see directories arduino and ros. 


All code has been written in C++ with exception of a few python scripts. 
For more details on this work, the thesis can be found [here](https://drive.google.com/open?id=0Bz08ndGq8YoiUlNWRVNkRlVtTEk). 


## Long story short:
This thesis was successful in creating an insight in deploying RL algorithms for actuated control systems.
- **Grid world**:
From modifying the reward signal and the exploration rate, both algorithms eventually converged to optimal behaviours. Q-learning took the riskier yet quicker path from the start to goal locations, whereas SARSA took the safer yet longer path.

- **Gazebo simulation**:
The Gazebo Simulator showed a promising application of the RL algorithms. Q-learning proved to be the quickest in responding to disturbances. SARSA proved to be the most adaptable where it performed vastly better when subjected to a rocky unknown terrain. However, the PID controller was by far the smoothest.

- **Robot**:
While the PID algorithm was very successful in balancing the robot, the RL algorithms were not. For the RL algorithms to become successful, the state space must encapsulate the actual dynamics of the robot more accurately. For example, a state space consisting of the pitch angle, pitch angular velocity, pitch angular acceleration, linear momentum, angular momentum, horizontal velocity and horizontal acceleration would have accurately described the dynamics of the robot at any point in time. However having such a large state space would have meant the agent would take must longer to converge to an optimal response. This is where Deep RL algorithms come into play, however this was beyond the scope of this thesis. 
Another important thing to mention is that the chassis of the robot could have been improved. During each algorithms training, various components of the chassis of the robot begun to move. This meant, the chasis weight distribution was changing meaning the model created by the algorithms would not have worked since it was made for a different weight distribution. 










