

#include "two_wheeled.h"

#define MAX_EPISODE 5000

using namespace std;

int main()
{
    // create main variables
    signed short int time_step, reward;
    unsigned int wins, loses, breakpoint = 0,wins_prev=0;
    float td_error, td_target, discount_factor, alpha, epsilon;
    char current_state, action, next_state, current_state_idx, next_state_idx, max_action_idx;
    std::vector<bool> available_actions(ACTIONS, false);
    std::vector<float> q_row(ACTIONS);

    // create object instances
    q_learning controller;
    two_wheeled env;

    srand(time(NULL));//seed the randomizer

    // rl variables (put in controller?)
    discount_factor = 0.3;
    alpha = 0.3;
    epsilon = 0.3;

    wins = 0;
    loses = 0;

    clock_t time_take_action, time_take_obs;


    for (int episode = 0; episode < MAX_EPISODE; episode++)
    {

        time_step = 0;
        current_state = 5;//start at 5 degs

        while(1)
        {
            //get all legal actions based on state
            available_actions = env.available_actions(current_state);

            //choose action based on policy
            current_state_idx = env.get_state_index(current_state);
            q_row = env.Q[current_state_idx];
            action = controller.choose_action(epsilon, available_actions, q_row);

            //take action
            time_take_action = clock(); //get time when action is initiated
            //next_state = env.take_action(action, current_state);

            //use delay between taking action and reading obs?
            //BOTH TIMES ARE GOING TO BE THE SAME...

            time_take_obs = clock();//get time when obs is taken
            //get new state

            next_state = env.next_state(action, current_state, available_actions);

            //get reward
            reward = env.get_reward(next_state);

            //TD update
            next_state_idx = env.get_state_index(next_state);
            max_action_idx = distance(env.Q[next_state_idx].begin(), max_element(env.Q[next_state_idx].begin(), env.Q[next_state_idx].end()));
            td_target = reward + discount_factor*env.Q[next_state_idx][max_action_idx];
            td_error = td_target - env.Q[current_state_idx][action];
            env.Q[current_state_idx][action]+= td_error*alpha;

            if (reward == 100)
            {
                wins++;
                break;
            }
            else if (reward == -100)
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
        if (episode % 10 == 0 && episode > 1)
        {
            //print stats
            cout<<"-------------------------------------------"<<endl;
            cout<<"Episode number: "<<episode<<" | ";
            cout<<"Time step: "<<time_step<<" | ";
            cout<<"Wins: "<<wins<<" | ";
            cout<<"Loses: "<<loses<<" | ";
            cout<<"Epsilon: "<<epsilon<<endl;
            breakpoint++;
        }
        //reduce exploration over time and when wins continually increase
        if (episode % 30 == 0 && wins > wins_prev*1.75)
        {
            if (epsilon > 0.06)//because c++ and floats are gay
            {
                epsilon-=0.05;
            }
            else
            {
                epsilon = 0;
            }
        }
    }
}

