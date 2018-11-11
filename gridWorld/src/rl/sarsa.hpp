
/**
	SARAS class declaration
	@author Alex Cornelio
*/

#ifndef SARSA_H
#define SARSA_H

#include "rl.hpp"
#include <iostream>
#include <stdlib.h>
#include <cmath>

class sarsa : public RL
{
public:
    sarsa();
    ~sarsa();

    char chooseAction(float epsilon, std::vector<bool> available_actions, std::vector<float> state_row);

};

#endif // SARSA_H
