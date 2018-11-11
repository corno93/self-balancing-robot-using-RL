#ifndef RL_H
#define RL_H

#include <vector>


/**
	Base class for all Rienforcement learning algorithms
*/
class RL
{
public:
    RL();
    ~RL();

    char policy(std::vector<char>);
    char virtual choose_action(float epsilon, std::vector<bool> available_actions, std::vector<float> state_row) = 0;
};

#endif // RL_H
