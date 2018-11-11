#ifndef gridWorld_H
#define gridWorld_H

#include <rl/environment.hpp>
#include <vector>
#include <stdlib.h>
#include <time.h>
#include <iostream>
#include <algorithm>

#define STATES 12
#define ACTIONS 4

#define REWARD 1000
#define PUNISHMENT -1000
#define STATE_TRANSITION_COST 0
#define GOAL_STATE 30
#define OBSTACLE_1_STATE 10
#define OBSTACLE_2_STATE 20

#define NOISEY_TRANS_PROB 20

/**
    Derive cliff word from environment class
*/
class gridWorld : public environment
{
public:

    // Q table
    std::vector<std::vector<float> > Q;

    gridWorld();
    ~gridWorld();

    std::vector<bool> availableActions(char s);
    char takeAction(char action, char current_state);
    char nextState(char action, char current_state, std::vector<bool> availiable_actions);
    signed short int getReward(char next_state);
    char getStateIndex(char current_state);

};

#endif // gridWorld_H
