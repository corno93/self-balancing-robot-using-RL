#include "cliff_world.h"
#include <cmath>

cliff_world::cliff_world()
    :Q(STATES, std::vector<float>(ACTIONS,0)) //create states rows and actions columns
{

}

cliff_world::~cliff_world()
{

}

std::vector<bool> cliff_world::available_actions(char s)
{
    //check boundaries. true for available action in that direction.
    // N, E, S, W is the order of actions
    std::vector<bool> actions(4, false);
    char s_x, s_y;

    //get each digit
    if (s >= 10)
    {
        s_x = s / 10;
        s_y = s % 10;
    }
    else
    {
        s_x = 0;
        s_y = s % 10;
    }

    //check north boundary
    if (s_y == 2)
    {
        actions[0] = false;
    }
    else
    {
        actions[0] = true;
    }
    //check east boundary
    if (s_x == 3)
    {
        actions[1] = false;
    }
    else
    {
        actions[1] = true;
    }
    //check southern boundary
    if (s_y == 0)
    {
        actions[2] = false;
    }
    else
    {
        actions[2] = true;
    }
    //check western boundary
    if (s_x == 0)
    {
        actions[3] = false;
    }
    else
    {
        actions[3] = true;
    }

    return actions;
}

char cliff_world::take_action(char action, char current_state)
{
    char delta_state;
    if (action == 0)
    {
        delta_state = 1;
    }
    else if(action == 1)
    {
        delta_state = 10;
    }
    else if(action == 2)
    {
        delta_state = -1;
    }
    else if (action == 3)
    {
        delta_state = -10;
    }
    return (current_state + delta_state);
}

char cliff_world::next_state(char action, char current_state, std::vector<bool> available_actions)
{
    // return the next state. take into account noisy transition probabilities
    //the chosen action will always have 80 % of resolving in the correct state. the remaining
    //avaliable options are divided equally and are chosen randomly if a random number is less than 20%...

    //get amount of available actions
    char noisey_idx,actual_next_state ;
    std::vector<int> available_idx;
    float next_state_probs;

    next_state_probs = fabs((rand()/(float)(RAND_MAX + 1)));    //random positive float between 0 and 1

    if (next_state_probs > 0.1)
    {
        //next state is not acted by noise
        actual_next_state = cliff_world::take_action(action, current_state);
        return actual_next_state;
    }
    else
    {
        //next state is noisy. pick other actions randomly
        //noisey_idx = rand()%(available_actions.size());
        //std::cout<<available_actions.size()<<std::endl;
        for (int i = 0; i < available_actions.size();i++)
        {
            if (available_actions[i] == true)
            {
                available_idx.push_back(i);
            }
        }
        noisey_idx = rand()%(available_idx.size());
        action = available_idx[noisey_idx];
        actual_next_state = cliff_world::take_action(action, current_state);
        return actual_next_state;
    }
}

signed short int cliff_world::get_reward(char next_state)
{
    signed short int reward;

    //punish for moving into obstacles
    if (next_state == 10 || next_state == 20)
    {
        reward = -1000;
    }
    //reward for moving into goal
    else if (next_state == 30)
    {
        reward = 1000;
    }
    else
    {
        reward = 0;
    }
    return reward;

}

char cliff_world::get_state_index(char state)
{
    char state_index;
    if(state == 0){
        state_index=0;
    }else if(state == 1){
        state_index = 1;
    }else if(state == 2){
        state_index = 2;
    }else if(state == 10){
        state_index = 3;
    }else if(state == 11){
        state_index = 4;
    }else if(state == 12){
        state_index = 5;
    }else if(state == 20){
        state_index = 6;
    }else if(state == 21){
        state_index = 7;
    }else if(state == 22){
        state_index = 8;
    }else if(state == 30){
        state_index = 9;
    }else if(state == 31){
        state_index = 10;
    }else if(state == 32){
        state_index = 11;
    }
    return state_index;
}
