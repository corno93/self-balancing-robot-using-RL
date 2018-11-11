#include "gridWorld.hpp"

#include <cmath>

/**
    Constructor
*/
gridWorld::gridWorld()
    :Q(STATES, std::vector<float>(ACTIONS,0)) //create states rows and actions columns
{

}

/**
    Destructor
*/
gridWorld::~gridWorld()
{

}

/**
    Return a vector of booleans for the actions available for the agent to take. 
    The agent cannot go beyond the grid's boards.
    N, E, S, W is the order of actions
*/
std::vector<bool> gridWorld::availableActions(char s)
{
    std::vector<bool> actions(4, false);
    char s_x, s_y;

    //get each digit of the agents state
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

/**
    Returns the state from taking the action
*/
char gridWorld::takeAction(char action, char current_state)
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

/**
    Return the next state from taking an action in the current state. 
    This method takes into account noisy state transistions. This means that the action taken has a 
    probability of NOISEY_TRANS_PROB to leading to an incorrect next state. 
*/
char gridWorld::nextState(char action, char current_state, std::vector<bool> availableActions)
{
    char noisey_idx,actual_next_state ;
    std::vector<int> available_idx;
    float next_state_probs;

    next_state_probs = fabs((rand()/(float)(RAND_MAX + 1)));    //random positive float between 0 and 1

    if (next_state_probs > NOISEY_TRANS_PROB/100)
    {
        //next state is not acted by noise
        actual_next_state = gridWorld::takeAction(action, current_state);
        return actual_next_state;
    }
    else
    {
        //next state is noisy. pick other true actions randomly and take that 
        for (int i = 0; i < availableActions.size();i++)
        {
            if (availableActions[i] == true)
            {
                available_idx.push_back(i);
            }
        }
        noisey_idx = rand()%(available_idx.size());
        action = available_idx[noisey_idx];
        actual_next_state = gridWorld::takeAction(action, current_state);
        return actual_next_state;
    }
}

/**
    Return the reward for the agents state transitions    
*/
signed short int gridWorld::getReward(char next_state)
{
    signed short int reward;

    //punish for moving into obstacles
    if (next_state == OBSTACLE_1_STATE || next_state == OBSTACLE_2_STATE)
    {
        reward = PUNISHMENT;
    }
    //reward for moving into goal
    else if (next_state == GOAL_STATE)
    {
        reward = REWARD;
    }
    // slightly punish for state transition
    else
    {
        reward = STATE_TRANSITION_COST;
    }
    return reward;

}

/**
    Return the state index
*/
char gridWorld::getStateIndex(char state)
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
