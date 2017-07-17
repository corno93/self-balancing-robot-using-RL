#include "two_wheeled.h"

two_wheeled::two_wheeled()
    :Q(STATES, std::vector<float>(ACTIONS,0)) //create states rows and actions columns
{

}

two_wheeled::~two_wheeled()
{

}


std::vector<bool> two_wheeled::available_actions(char s)
{

}

char two_wheeled::take_action(char action, char current_state)
{

}

char two_wheeled::next_state(char action, char current_state, std::vector<bool> availiable_actions)
{

}

signed short int two_wheeled::get_reward(char next_state)
{

}

char two_wheeled::get_state_index(char current_state)
{

}
