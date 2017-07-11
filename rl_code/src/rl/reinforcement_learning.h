#ifndef REINFORCEMENT_LEARNING_H
#define REINFORCEMENT_LEARNING_H

#include <vector>


class reinforcement_learning
{
public:
    reinforcement_learning();//float ep);
    ~reinforcement_learning();
  //  float epsilon;

  //  std::vector<std::vector<int> > board;
  //  std::vector<std::vector<float>> Q(5, std::vector<float>(5,0));

    char policy(std::vector<char>);
    char virtual choose_action(char s, std::vector<bool> available_actions, std::vector<float> state_row) = 0;
};

#endif // REINFORCEMENT_LEARNING_H
