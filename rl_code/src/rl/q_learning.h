#ifndef Q_LEARNING_H
#define Q_LEARNING_H

#include "reinforcement_learning.h"
#include <iostream>
#include <stdlib.h>
#include <cmath>

class q_learning : public reinforcement_learning
{
public:
    q_learning();//float ep);
    ~q_learning();

    char choose_action(char s, std::vector<bool> available_actions, std::vector<float> state_row);


};

#endif // Q_LEARNING_H
