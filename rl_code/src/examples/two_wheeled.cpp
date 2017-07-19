#include "two_wheeled.h"
#include <cmath>

two_wheeled::two_wheeled()
    :Q(STATES, std::vector<float>(ACTIONS,0)) //create states rows and actions columns
{

}

two_wheeled::~two_wheeled()
{

}


std::vector<bool> two_wheeled::available_actions(char s)
{
    std::vector<bool> actions(ACTIONS,false);
    //available actions are those in the same half the state is
    if(s<=2)
    {
        actions[0] = true;
        actions[1] = true;
        actions[2] = true;
        actions[3] = false;
        actions[4] = false;
        actions[5] = false;
    }
    else if (s >=3)
    {
        actions[0] = false;
        actions[1] = false;
        actions[2] = false;
        actions[3] = true;
        actions[4] = true;
        actions[5] = true;
    }
    return actions;
}

char two_wheeled::take_action(char action, char current_state)
{
    char delta_state;
    if (action == 0)
    {
        delta_state = 30;
    }else if (action == 1)
    {
        delta_state = 20;
    }else if (action == 2)
    {
        delta_state = 10;
    }else if (action == 3)
    {
        delta_state = -10;
    }else if (action == 4)
    {
        delta_state = -20;
    }else if (action == 5)
    {
        delta_state = -30;
    }
    return current_state+delta_state;
}

char two_wheeled::next_state(char action, char current_state, std::vector<bool> available_actions)
{
    //transition probabilities

        // return the next state. take into account noisy transition probabilities
        //the chosen action will always have 80 % of resolving in the correct state. the remaining
        //avaliable options are divided equally and are chosen randomly if a random number is less than 20%...

        //get amount of available actions
        char noisey_idx,actual_next_state ;
        std::vector<int> available_idx;
        float next_state_probs;

        next_state_probs = fabs((rand()/(float)(RAND_MAX + 1)));    //random positive float between 0 and 1

        if (next_state_probs > 0.2)
        {
            //next state is not acted by noise
            actual_next_state = two_wheeled::take_action(action, current_state);
            return actual_next_state;
        }
        else
        {
            //next state is noisy. pick other actions randomly
            noisey_idx = rand()%(available_actions.size());
            action = available_actions[noisey_idx];
            actual_next_state = two_wheeled::take_action(action, current_state);
            return actual_next_state;
        }
}

signed short int two_wheeled::get_reward(char next_state)
{
    signed short int reward;
    if (abs(next_state) > 30 )
    {
        reward = -100;
    }
    else if (abs(next_state) < 30 && abs(next_state) > 20)
    {
        reward = 1;
    }
    else if (abs(next_state) < 20 && abs(next_state) > 10)
    {
        reward = 5;
    }
    else if (abs(next_state) < 10 && abs(next_state) >= 0)
    {
        reward = 100;
    }

    return reward;
}

char two_wheeled::get_state_index(char current_state)
{
    char state_idx;
    if (current_state < -30)
    {
        state_idx = 0;
    }else if (current_state < -20 && current_state > -30)
    {
        state_idx = 1;
    }else if (current_state < -10 && current_state > -20)
    {
        state_idx = 2;
    }else if (current_state < 0 && current_state > -10)
    {
        state_idx = 3;
    }else if (current_state < 10 && current_state > 0)
    {
        state_idx = 4;
    }else if (current_state < 20 && current_state > 10)
    {
        state_idx = 5;
    }else if (current_state < 30 && current_state > 20)
    {
        state_idx = 6;
    }else if (current_state > 30)
    {
        state_idx = 7;
    }
    return state_idx;
}
