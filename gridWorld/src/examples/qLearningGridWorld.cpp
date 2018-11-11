/**
    This script runs q-learning in the gridworld environment. 
    The agents state is defined by a 2 digit number where the first digit is its x coordinate and the second is its
    y coordinate in the grid. 00 is the bottom left. 
    Please see this thesis for more information on how this algorithm works.

    @author Alex Cornelio
*/



#include <iostream>
#include <vector>

#include "gridWorld.hpp"

#include <rl/rl.h>
#include <rl/q_learning.hpp>

#define MAX_EPISODE 100
// agent parameters
#define DISCOUNT_FACTOR 0.5
#define ALPHA 0.5
#define EPSILON 0.5

using namespace std;

int main()
{
    // create main variables
    signed short int time_step, reward;
    unsigned int wins, loses;
    float td_error, td_target, epsilon;
    char current_state, action, next_state, current_state_idx, next_state_idx, max_action_idx;
    std::vector<bool> availableActions(4, false);
    std::vector<float> q_row(4);

    // create object instances
    q_learning controller;
    gridWorld env;

    srand(time(NULL));//seed the randomizer

    wins = 0;
    loses = 0;
    epsilon = EPSILON;

    for (int episode = 0; episode < MAX_EPISODE; episode++)
    {

        // initalise system
        time_step = 0;
        current_state = 0;

        // continue forever until the agent has reached goal state or failed
        while(1)
        {
            //get all legal actions based on state
            available_acctions = env.availableActions(current_state);

            //choose action based on policy
            current_state_idx = env.getStateIndex(current_state);
            action = controller.chooseAction(epsilon, available_acctions, env.Q[current_state_idx]);

            //take action to get nextstate
            next_state = env.takeAction(action, current_state, available_acctions);

            //get reward
            reward = env.getReward(next_state);

            //TD update
            next_state_idx = env.getStateIndex(next_state);            
            max_action_idx = distance(env.Q[next_state_idx].begin(), max_element(env.Q[next_state_idx].begin(), env.Q[next_state_idx].end()));
            td_target = reward + DISCOUNT_FACTOR*env.Q[next_state_idx][max_action_idx];
            td_error = td_target - env.Q[current_state_idx][action];
            env.Q[current_state_idx][action]+= td_error*ALPHA;
            
            // update wins, counts, timesteps and states
            if (reward == REWARD)
            {
                wins++;
                break;
            }
            else if (reward == PUNISHMENT)
            {
                loses++;
                break;
            }
            else
            {
                time_step++;
                current_state = next_state;
            }
        }

        //print stats 
        cout<<"-------------------------------------------"<<endl;
        cout<<"Episode number: "<<episode<<" | ";
        cout<<"Time step: "<<time_step<<" | ";
        cout<<"Wins: "<<wins<<" | ";
        cout<<"Loses: "<<loses<<" | ";
        cout<<"EPSILON: "<<epsilon<<endl;

        
        //reduce exploration over time
        if (episode % 10 == 0 && episode > 1 && epsilon > 0.0)
        {
            epsilon-=0.2;
        }
    }
}
