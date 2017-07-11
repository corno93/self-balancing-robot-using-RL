
#include <iostream>
#include <vector>

#include "cliff_world.h"

#include <rl/reinforcement_learning.h>
#include <rl/q_learning.h>

#define MAX_EPISODE 50

using namespace std;

int main()
{
    // create main variables
    signed short int time_step, reward;
    unsigned int wins, loses;
    float td_error, td_target, discount_factor, alpha;
    char current_state, goal_state, action, next_state, current_state_idx, next_state_idx, max_action_idx;
    std::vector<bool> available_actions(4, false);
    std::vector<float> state_row(4);

    // create object instances
    q_learning controller;
    cliff_world env;

    discount_factor = 0.3;
    alpha = 0.3;


    for (int episode = 0; episode < MAX_EPISODE; episode++)
    {
        cout<<"episode number is: "<<episode<<endl;
        // reset agent and world

        time_step = 0;
        current_state = 00;
        goal_state = 30;
        wins = 0;
        loses = 0;

        while(1)
        {
            cout<<"time_step number is: "<< time_step <<endl;

            //get all legal actions based on state
            available_actions = env.available_actions(current_state);

            //choose action based on policy
            current_state_idx = env.get_state_index(current_state);
            state_row = env.Q[current_state_idx];
            action = controller.choose_action(current_state, available_actions, state_row);

            //take action
            //env.take_action(action);

            //get next state. incorporates transition probs.
            next_state = env.next_state(action, current_state, available_actions);

            //get reward
            reward = env.get_reward(next_state);

            //TD update
            next_state_idx = env.get_state_index(next_state);
            max_action_idx = distance(env.Q[next_state_idx].begin(), max_element(env.Q[next_state_idx].begin(), env.Q[next_state_idx].end()));
            td_target = reward + discount_factor*env.Q[next_state_idx][max_action_idx];
            td_error = td_target - env.Q[current_state_idx][action];
            env.Q[current_state_idx][action]+= td_error*alpha;

            if (reward == 1000)
            {
                wins++;
                break;
            }
            else if (reward == -1000)
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
        cout<<"-------------------------"<<endl;
        cout<<"Episode number: "<<episode<<" ";
        cout<<"Wins: "<<wins<<" ";
        cout<<"Loses: "<<loses<<endl;

    }
}
