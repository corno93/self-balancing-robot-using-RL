#ifndef ENVIRONMENT_H
#define ENVIRONMENT_H

#include <vector>


class environment
{
public:

    environment();  //constructor
    ~environment(); //deconstructor

    virtual std::vector<bool> available_actions(char s) = 0;
    virtual char take_action(char action, char current_state) = 0;
    virtual char next_state(char action, char current_state, std::vector<bool> availiable_actions) = 0;
    virtual signed short int get_reward(char next_state) = 0;


};

#endif // ENVIRONMENT_H
