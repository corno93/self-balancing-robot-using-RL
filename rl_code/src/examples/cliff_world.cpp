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

char cliff_world::next_state(char action, char current_state, std::vector<bool> availiable_actions)
{
    // return the next state. take into account noisy transition probabilities
    //the chosen action will always have 70 % of been resulting in the correct state. the remaining
    //percetange is divided equally between other available states...

    //get amount of available actions
    int num_availiable_actions;
    num_availiable_actions = std::count(availiable_actions.begin(), availiable_actions.end(), true);
    float probability_dist[ACTIONS];
    char actual_next_state;

    //split probabilities. choose. calc next state based on choice

    for (int i = 0; i < ACTIONS; i++)
    {
        if (availiable_actions[i] == true)
        {
           if (i == action)
           {
               probability_dist[i] = 0.8;
           }
           else
           {
               probability_dist[i] = 0.2/(num_availiable_actions-1);
           }
        }
        else
        {
            probability_dist[i] = 0;
        }
    }

    float probs = fabs((rand()/(float)(RAND_MAX + 1)));
    std::cout<<"Random probs for moving to next state is "<<probs<<std::endl;
    char actual_action;
    for (int i = 0; i < ACTIONS; i ++)
    {
        if (probs <= probability_dist[i])
        {
            actual_action = i;
        }
        else
        {
            probs = probs - probability_dist[i];
        }
    }

    actual_next_state = cliff_world::take_action(actual_action, current_state);
    return actual_next_state;

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
        reward = 1;
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
