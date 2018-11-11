/**
	Environment class methods. All are virtual and will be implemented by derived classes
	@author Alex Cornelio
*/

#include "environment.h"

environment::environment()
{
}

environment::~environment()
{

}

std::vector<bool> environment::availableActions(char s)
{

}

char environment::takeAction(char action, char current_state)
{

}

char environment::nextState(char action, char current_state,std::vector<bool> availiable_actions)
{

}

signed short int environment::getReward(char next_state)
{

}
