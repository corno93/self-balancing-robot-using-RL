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
    std::vector<bool> actions(4,false);
    //available actions are those in the same half the state is
    if(s<=2)
    {
        actions[0] = true;
        actions[1] = true;
        actions[2] = true;
        actions[3] = false;
        actions[4] = false;
        actions[5] = false;
    }
    else if (s >=3)
    {
        actions[0] = false;
        actions[1] = false;
        actions[2] = false;
        actions[3] = true;
        actions[4] = true;
        actions[5] = true;
    }
    return actions;
}

char two_wheeled::take_action(char action, char current_state)
{

}

char two_wheeled::next_state(char action, char current_state, std::vector<bool> availiable_actions)
{

}

signed short int two_wheeled::get_reward(char next_state)
{
    signed short int reward;

}

char two_wheeled::get_state_index(char current_state)
{

}
