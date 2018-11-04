#ifndef SARSA_H
#define SARSA_H

#include "reinforcement_learning.h"
#include <iostream>
#include <stdlib.h>
#include <cmath>

class sarsa : public reinforcement_learning
{
public:
    sarsa();
    ~sarsa();

    char choose_action(float epsilon, std::vector<bool> available_actions, std::vector<float> state_row);

};

#endif // SARSA_H
