#ifndef CLIFF_WORLD_H
#define CLIFF_WORLD_H

#include <rl/environment.h>
#include <vector>
#include <stdlib.h>
#include <time.h>
#include <iostream>
#include <algorithm>

#define STATES 12
#define ACTIONS 4

class cliff_world : public environment
{
public:
    char size_H = 3;
    char size_W = 4;

    char obstacle_one = 10;
    char obstacle_two = 20;

    std::vector<std::vector<float> > Q;   //Q(s,a) table

    cliff_world();
    ~cliff_world();
    //void transition_probs(char s, char a);

    std::vector<bool> available_actions(char s);
    char take_action(char action, char current_state);
    char next_state(char action, char current_state, std::vector<bool> availiable_actions);
    signed short int get_reward(char next_state);
    char get_state_index(char current_state);

};

#endif // CLIFF_WORLD_H
