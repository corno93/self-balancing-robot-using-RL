/**
	Q-Learning class declaration
	@author Alex Cornelio
*/

#ifndef Q_LEARNING_H
#define Q_LEARNING_H

#include "rl.hpp"
#include <iostream>
#include <stdlib.h>
#include <cmath>

class q_learning : public reinforcement_learning
{
public:
    q_learning();//float ep);
    ~q_learning();

    char choose_action(float epsilon, std::vector<bool> available_actions, std::vector<float> state_row);


};

#endif // Q_LEARNING_H
