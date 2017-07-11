#include "q_learning.h"
#include "environment.h"



#include <algorithm>

q_learning::q_learning()//float ep)
  //  :epsilon(ep)
{
   // q_learning.epsilon = epsilon;
}

q_learning::~q_learning()
{

}



char q_learning::choose_action(char s, std::vector<bool> available_actions, std::vector<float> q_row)
{
    float random_num;
    int random_choice, random_action;
    int max_q;
    std::vector<int> max_indexs;
    std::vector<int> true_max_actions, true_actions;

    //first remove all falses from list. do this by making new vector true_actions with elements representing indexs of avaliable actions
    for (int j = 0; j < available_actions.size(); j ++)
    {
        if (available_actions[j] == true)
        {
            true_actions.push_back(j);
        }
    }

    random_num = fabs((rand()/(float)(RAND_MAX + 1)));
    std::cout<<"The random num for actions is "<<random_num<<std::endl;

    if (random_num < 0.3)//q_learning.epsilon)
    {
        //pick randomly:
        random_choice = ((int)rand()%(true_actions.size()+1));
        std::cout<<"The random choice for random actions is"<<random_choice<<std::endl;
        random_action = true_actions[random_choice];
        return random_action;

    }
    else
    {
        //pick best:

        //need to pick maximum true action..


        //get max value. then get index's of all max elements
        max_q = *std::max_element(q_row.begin(),q_row.end());
        for (int i = 0; i < q_row.size(); i++)
        {
            if (max_q == q_row[i])
            {
                max_indexs.push_back(i);
            }
        }
        //ERROR - WHEN THE BEST VALUES ARE ALL FOR FALSE ACTIONS...
        //check that indexs are a true action. insert true max index values into true_max_actions
        for (int i = 0; i < max_indexs.size();i++)
        {
            if (available_actions[max_indexs[i]] == true)
            {
                true_max_actions.push_back(max_indexs[i]);
            }
        }
        //randomly pick if there are more than one option
        if (true_max_actions.size() == 0)
        {
            // order. then get idex of first
        }
        else if (true_max_actions.size() == 1)
        {
            return true_max_actions[0];
        }
        else if (true_max_actions.size() > 1)
        {

            random_choice = ((int)rand()%(true_max_actions.size()+1));
            std::cout<<"Random choice for picking best action is "<<random_choice<<std::endl;
            return true_max_actions[random_choice];
        }
    }

}
