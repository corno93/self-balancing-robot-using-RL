#ifndef TWO_WHEELED_H
#define TWO_WHEELED_H

#include <rl/environment.h>
#include <vector>
#include <stdlib.h>
#include <time.h>
#include <iostream>
#include <algorithm>

#define STATES 6
#define ACTIONS 6

class two_wheeled : public environment
{
public:

    std::vector<std::vector<float> > Q;   //Q(s,a) table

    two_wheeled();
    ~two_wheeled();

    std::vector<bool> available_actions(char s);
    char take_action(char action, char current_state);
    char next_state(char action, char current_state, std::vector<bool> availiable_actions);
    signed short int get_reward(char next_state);
    char get_state_index(char current_state);

};

#endif // TWO_WHEELED_H
