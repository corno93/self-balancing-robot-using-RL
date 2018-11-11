
/**
    Q-Learning algorithm methods
    @author Alex Cornelio
*/

#include "q_learning.hpp"
#include "environment.hpp"

#include <algorithm>

/**
    Constructor
*/
q_learning::q_learning()
{
}

/**
    Desctructor
*/
q_learning::~q_learning()
{
}


/**
    Choose an action. 
    The agent will explore, meaning it will choose a random legal action is a generated random number is less than epsilon. 
    The agent will exploit, meaning it will choose the best action based on its learnt Q matrix.
*/
char q_learning::chooseAction(float epsilon, std::vector<bool> available_actions, std::vector<float> q_row)
{
    float random_num;
    int random_choice, random_action;
    float max_q;
    std::vector<int> max_indexs, true_actions;
    std::vector<float> q_row_true;

    //first remove all falses from list. do this by making new vector true_actions with elements representing indexs of avaliable actions
    for (int j = 0; j < available_actions.size(); j ++)
    {
        if (available_actions[j] == true)
        {
            true_actions.push_back(j);
        }
    }

    // explore or exploit
    random_num = fabs((rand()/(float)(RAND_MAX + 1))); //produce random positive float between 0 and 1
    if (random_num < epsilon)
    {
        //pick randomly:
        random_choice = rand()%(true_actions.size());   //produe random positive int between 0 and true_actions size
        random_action = true_actions[random_choice];
        return random_action;

    }
    else
    {
        // pick best action
        for (int i = 0; i < true_actions.size();i++)
        {
            q_row_true.push_back(q_row[true_actions[i]]);//get true q values
        }

        max_q = *std::max_element(q_row_true.begin(),q_row_true.end());

        //check for multiple best actions
        for (int i = 0; i < q_row_true.size();i++)
        {
            if (max_q == q_row_true[i])
            {
                max_indexs.push_back(i);
            }
        }

        //randomly pick if there are more than one best action
        if (max_indexs.size() == 1)
        {
            return true_actions[max_indexs[0]];
        }
        else if (max_indexs.size() > 1)
        {
            random_choice = rand()%(max_indexs.size());
            return true_actions[max_indexs[random_choice]];
        }
    }

}
