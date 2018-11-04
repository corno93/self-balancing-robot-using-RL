#ifndef TWO_WHEELED_H
#define TWO_WHEELED_H

#include <vector>
#include <stdlib.h>
#include <time.h>
#include <iostream>
#include <algorithm>
#include <cmath>


#include <rl/reinforcement_learning.h>
#include <rl/q_learning.h>
#include <rl/environment.h>


#define STATES 8
#define ACTIONS 6

/*Actions:
 * 1  50 deg/sec
 * 2  30 deg/sec
 * 3  10 deg/sec
 * 4 -10 deg/sec
 * 5 -30 deg/sec
 * 6 -50 deg/sec
 */

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
