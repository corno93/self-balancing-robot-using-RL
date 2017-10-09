/*********************************************************************
 *  Copyright (c) 2015 Robosavvy Ltd.
 *  Author: Vitor Matos
 *
 *  Based on diff_drive_plugin
 *
 *  MODIFICATION: 
 *  - control segway about pitch angle using a RL q-learning controller

 *********************************************************************/

#include "gazebo_rsv_balance/gazebo_rsv_balance.h"

#include <string>
#include <map>
#include <fstream>
#include <sstream>
#include <iostream>
#include <ros/ros.h>
#include <sdf/sdf.hh>

#include <vector>
#include <time.h>
#include <cmath>
#include <algorithm>

//read model?
#define MODEL_READ 1


#define REFERENCE_PITCH 0.0
#define PITCH_THRESHOLD 3.5
#define RL_DELTA 0.04
#define FREQ 25
#define ACTIONS 7
//char actions[ACTIONS] = {-30,-10,0,10,30};
char actions[ACTIONS] = {-53, -26, -13, 0, 13, 26, 53};	//torque of 3 recovers falling robot at 3 degreees
#define WHEEL_RADIUS 0.19
#define MAX_EPISODE 60

// 2D state space
#define STATE_NUM_PHI 9
#define STATE_NUM_PHI_D 11
float phi_states[STATE_NUM_PHI] = {-3, -2, -1, -0.5, 0, 0.5, 1, 2, 3};
float phi_d_states[STATE_NUM_PHI_D] = {-5, -4, -3, -2, -1, 0, 1, 2, 3, 4, 5};



class reinforcement_learning
{
  public:
    float alpha;
    int episode_num;
    int time_steps;
    int wins;
    int loses;
    float discount_factor;
    float epsilon;
    float pitch;
    float pitch_dot;
    float prev_pitch;
    char current_state;
    char next_state;
    rsv_balance_msgs::State msg;
    char action;
    char action_idx;
    float reward_per_ep;
    int disturbance_cntr;

    reinforcement_learning();
    ~reinforcement_learning();

    std::vector<std::vector<float> > Q;

    char virtual choose_action(char) = 0;
    void TD_update(char, char, char, float);
    char get_state(float, float);
    //char get_next_state(float,float, char);
    float get_reward(char);
    void read_model(void);
};

reinforcement_learning::reinforcement_learning()
  :  Q((STATE_NUM_PHI+1)*(STATE_NUM_PHI_D+1), std::vector<float>(ACTIONS,0)), 
     episode_num(0), time_steps(0), wins(0),
     loses(0), discount_factor(0.3), alpha(0.4),
     epsilon(0.6), pitch_dot(0.0), prev_pitch(0.0),
     reward_per_ep(0.0), disturbance_cntr(0)
{
	if (MODEL_READ == 1){
	read_model();}
}

reinforcement_learning::~reinforcement_learning()
{
}

float reinforcement_learning::get_reward(char next_state)
{
  float squared_error_pitch = pow((pitch - REFERENCE_PITCH),2);
  float squared_error_pitch_dot = pow((pitch_dot - 0), 2);

  if (pitch_dot < 0 && pitch < REFERENCE_PITCH)
    squared_error_pitch_dot = -squared_error_pitch_dot;
  else if (pitch_dot > 0 && pitch > REFERENCE_PITCH)
    squared_error_pitch_dot = -squared_error_pitch_dot;
  
  return (-squared_error_pitch + squared_error_pitch_dot); 
}

char reinforcement_learning::choose_action(char)
{
}

void reinforcement_learning::TD_update(char curr_state, char action, char next_state, float reward)
{
  int max_action_idx;
  float td_target;
  float td_error;
  float Q_val;
  
  // get index value of Q next_state row with max reward value
  max_action_idx = distance(Q[next_state].begin(), max_element(Q[next_state].begin(), Q[next_state].end()));
  // compute update and write to Q at current state
  td_target = reward + discount_factor*Q[next_state][max_action_idx];
  td_error = td_target - Q[curr_state][action];
  Q[curr_state][action]+= td_error*alpha;
 
  // add more data 
  msg.max_action_idx = max_action_idx;
  msg.td_target = td_target;
  msg.td_error = td_error;
  msg.td_update = Q[curr_state][action];
  msg.alpha = alpha;
  msg.discount_factor = discount_factor;    
}

char reinforcement_learning::get_state(float pitch, float pitch_dot)
{
  char i, j;
  i = 0;
  j = 0;

  for (char phi_idx = 0; phi_idx < STATE_NUM_PHI; phi_idx++)
  {
    if (pitch <= phi_states[phi_idx])
    {
      i = phi_idx;
      break;
    }else if (pitch > phi_states[STATE_NUM_PHI - 1])
    {
      i = STATE_NUM_PHI;
      break;
     }
   }

  for (char phi_d_idx = 0; phi_d_idx < STATE_NUM_PHI_D; phi_d_idx++)
  {
    if (pitch_dot <= phi_d_states[phi_d_idx])
    {
      j = phi_d_idx;
      break;
    }else if (pitch_dot > phi_d_states[STATE_NUM_PHI_D - 1])
    {
      j = STATE_NUM_PHI_D;
      break;
    }
 }
 
  return( j + (STATE_NUM_PHI_D + 1) * i);
}


void reinforcement_learning::read_model(void)
{
	
}
/*Q[0][0] = -21.44594383239746;
Q[0][1] =-20.235382080078125;
Q[0][2] =-26.137907028198242;
Q[0][3] =0.0;
Q[0][4] =0.0;
Q[0][5] = 0.0;
Q[0][6] = 0.01;
Q[1][0] = -21.44594383239746;
Q[1][1] =-20.235382080078125;
Q[1][2] =-26.137907028198242;
Q[1][3] =0.0;
Q[1][4] =0.0;
Q[1][5] = 0.0;
Q[1][6] = 0.01;
Q[2][0] = -21.44594383239746;
Q[2][1] =-20.235382080078125;
Q[2][2] =-26.137907028198242;
Q[2][3] =0.0;
Q[2][4] =0.0;
Q[2][5] = 0.0;
Q[2][6] = 0.01;
Q[3][0] = -21.44594383239746;
Q[3][1] =-20.235382080078125;
Q[3][2] =-26.137907028198242;
Q[3][3] =0.0;
Q[3][4] =0.0;
Q[3][5] = 0.0;
Q[3][6] = 0.01;
Q[4][0] = -21.44594383239746;
Q[4][1] =-20.235382080078125;
Q[4][2] =-26.137907028198242;
Q[4][3] =0.0;
Q[4][4] =0.0;
Q[4][5] = 0.0;
Q[4][6] = 0.01;
Q[5][0] = -21.44594383239746;
Q[5][1] =-20.235382080078125;
Q[5][2] =-26.137907028198242;
Q[5][3] =0.0;
Q[5][4] =0.0;
Q[5][5] = 0.0;
Q[5][6] = 0.01;
Q[6][0] = -21.44594383239746;
Q[6][1] =-20.235382080078125;
Q[6][2] =-26.137907028198242;
Q[6][3] =0.0;
Q[6][4] =0.0;
Q[6][5] = 0.0;
Q[6][6] = 0.01;
Q[7][0] = -21.44594383239746;
Q[7][1] =-20.235382080078125;
Q[7][2] =-26.137907028198242;
Q[7][3] =0.0;
Q[7][4] =0.0;
Q[7][5] = 0.0;
Q[7][6] = 0.01;
Q[8][0] = -21.44594383239746;
Q[8][1] =-20.235382080078125;
Q[8][2] =-26.137907028198242;
Q[8][3] =0.0;
Q[8][4] =0.0;
Q[8][5] = 0.0;
Q[8][6] = 0.01;
Q[9][0] = -21.44594383239746;
Q[9][1] =-20.235382080078125;
Q[9][2] =-26.137907028198242;
Q[9][3] =0.0;
Q[9][4] =0.0;
Q[9][5] = 0.0;
Q[9][6] = 0.01;
Q[10][0] = -21.44594383239746;
Q[10][1] =-20.235382080078125;
Q[10][2] =-26.137907028198242;
Q[10][3] =0.0;
Q[10][4] =0.0;
Q[10][5] = 0.0;
Q[10][6] = 0.01;
Q[11][0] = -21.44594383239746;
Q[11][1] =-20.235382080078125;
Q[11][2] =-26.137907028198242;
Q[11][3] =0.0;
Q[11][4] =0.0;
Q[11][5] = 0.0;
Q[11][6] = 0.01;
Q[12][0] = -21.44594383239746;
Q[12][1] =-20.235382080078125;
Q[12][2] =-26.137907028198242;
Q[12][3] =0.0;
Q[12][4] =0.0;
Q[12][5] = 0.0;
Q[12][6] = 0.01;
Q[13][0] = -21.44594383239746;
Q[13][1] =-20.235382080078125;
Q[13][2] =-26.137907028198242;
Q[13][3] =0.0;
Q[13][4] =0.0;
Q[13][5] = 0.0;
Q[13][6] = 0.01;
Q[14][0] = -21.44594383239746;
Q[14][1] =-20.235382080078125;
Q[14][2] =-26.137907028198242;
Q[14][3] =0.0;
Q[14][4] =0.0;
Q[14][5] = 0.0;
Q[14][6] = 0.01;
Q[15][0] = -21.44594383239746;
Q[15][1] =-20.235382080078125;
Q[15][2] =-26.137907028198242;
Q[15][3] =0.0;
Q[15][4] =0.0;
Q[15][5] = 0.0;
Q[15][6] = 0.01;
Q[16][0] = -21.44594383239746;
Q[16][1] =-20.235382080078125;
Q[16][2] =-26.137907028198242;
Q[16][3] =0.0;
Q[16][4] =0.0;
Q[16][5] = 0.0;
Q[16][6] = 0.01;
Q[17][0] = -21.44594383239746;
Q[17][1] =-20.235382080078125;
Q[17][2] =-26.137907028198242;
Q[17][3] =0.0;
Q[17][4] =0.0;
Q[17][5] = 0.0;
Q[17][6] = 0.01;
Q[18][0] = -21.44594383239746;
Q[18][1] =-20.235382080078125;
Q[18][2] =-26.137907028198242;
Q[18][3] =0.0;
Q[18][4] =0.0;
Q[18][5] = 0.0;
Q[18][6] = 0.01;
Q[19][0] = -21.44594383239746;
Q[19][1] =-20.235382080078125;
Q[19][2] =-26.137907028198242;
Q[19][3] =0.0;
Q[19][4] =0.0;
Q[19][5] = 0.0;
Q[19][6] = 0.01;
Q[20][0] = -21.44594383239746;
Q[20][1] =-20.235382080078125;
Q[20][2] =-26.137907028198242;
Q[20][3] =0.0;
Q[20][4] =0.0;
Q[20][5] = 0.0;
Q[20][6] = 0.01;
Q[21][0] = -21.44594383239746;
Q[21][1] =-20.235382080078125;
Q[21][2] =-26.137907028198242;
Q[21][3] =0.0;
Q[21][4] =0.0;
Q[21][5] = 0.0;
Q[21][6] = 0.01;
Q[22][0] = -21.44594383239746;
Q[22][1] =-20.235382080078125;
Q[22][2] =-26.137907028198242;
Q[22][3] =0.0;
Q[22][4] =0.0;
Q[22][5] = 0.0;
Q[22][6] = 0.01;
Q[23][0] = -21.44594383239746;
Q[23][1] =-20.235382080078125;
Q[23][2] =-26.137907028198242;
Q[23][3] =0.0;
Q[23][4] =0.0;
Q[23][5] = 0.0;
Q[23][6] = 0.01;
Q[24][0] = -21.44594383239746;
Q[24][1] =-20.235382080078125;
Q[24][2] =-26.137907028198242;
Q[24][3] =0.0;
Q[24][4] =0.0;
Q[24][5] = 0.0;
Q[24][6] = 0.01;
Q[25][0] = -21.44594383239746;
Q[25][1] =-20.235382080078125;
Q[25][2] =-26.137907028198242;
Q[25][3] =0.0;
Q[25][4] =0.0;
Q[25][5] = 0.0;
Q[25][6] = 0.01;
Q[26][0] = -21.44594383239746;
Q[26][1] =-20.235382080078125;
Q[26][2] =-26.137907028198242;
Q[26][3] =0.0;
Q[26][4] =0.0;
Q[26][5] = 0.0;
Q[26][6] = 0.01;
Q[27][0] = -21.44594383239746;
Q[27][1] =-20.235382080078125;
Q[27][2] =-26.137907028198242;
Q[27][3] =0.0;
Q[27][4] =0.0;
Q[27][5] = 0.0;
Q[27][6] = 0.01;
Q[28][0] = -21.44594383239746;
Q[28][1] =-20.235382080078125;
Q[28][2] =-26.137907028198242;
Q[28][3] =0.0;
Q[28][4] =0.0;
Q[28][5] = 0.0;
Q[28][6] = 0.01;
Q[29][0] = -21.44594383239746;
Q[29][1] =-20.235382080078125;
Q[29][2] =-26.137907028198242;
Q[29][3] =0.0;
Q[29][4] =0.0;
Q[29][5] = 0.0;
Q[29][6] = 0.01;
Q[30][0] = -21.44594383239746;
Q[30][1] =-20.235382080078125;
Q[30][2] =-26.137907028198242;
Q[30][3] =0.0;
Q[30][4] =0.0;
Q[30][5] = 0.0;
Q[30][6] = 0.01;
Q[31][0] = -21.44594383239746;
Q[31][1] =-20.235382080078125;
Q[31][2] =-26.137907028198242;
Q[31][3] =0.0;
Q[31][4] =0.0;
Q[31][5] = 0.0;
Q[31][6] = 0.01;
Q[32][0] = -21.44594383239746;
Q[32][1] =-20.235382080078125;
Q[32][2] =-26.137907028198242;
Q[32][3] =0.0;
Q[32][4] =0.0;
Q[32][5] = 0.0;
Q[32][6] = 0.01;
Q[32][0] = -21.44594383239746;
Q[33][1] =-20.235382080078125;
Q[33][2] =-26.137907028198242;
Q[33][3] =0.0;
Q[33][4] =0.0;
Q[33][5] = 0.0;
Q[33][6] = 0.01;
Q[34][0] = -21.44594383239746;
Q[34][1] =-20.235382080078125;
Q[34][2] =-26.137907028198242;
Q[34][3] =0.0;
Q[34][4] =0.0;
Q[34][5] = 0.0;
Q[34][6] = 0.01;
Q[35][0] = -21.44594383239746;
Q[35][1] =-20.235382080078125;
Q[35][2] =-26.137907028198242;
Q[35][3] =0.0;
Q[35][4] =0.0;
Q[35][5] = 0.0;
Q[35][6] = 0.01;
Q[36][0] = -21.44594383239746;
Q[36][1] =-20.235382080078125;
Q[36][2] =-26.137907028198242;
Q[36][3] =0.0;
Q[36][4] =0.0;
Q[36][5] = 0.0;
Q[36][6] = 0.01;
Q[37][0] = -21.44594383239746;
Q[37][1] =-20.235382080078125;
Q[37][2] =-26.137907028198242;
Q[37][3] =0.0;
Q[37][4] =0.0;
Q[37][5] = 0.0;
Q[37][6] = 0.01;
Q[38][0] = -21.44594383239746;
Q[38][1] =-20.235382080078125;
Q[38][2] =-26.137907028198242;
Q[38][3] =0.0;
Q[38][4] =0.0;
Q[38][5] = 0.0;
Q[38][6] = 0.01;
Q[39][0] = -21.44594383239746;
Q[39][1] =-20.235382080078125;
Q[39][2] =-26.137907028198242;
Q[39][3] =0.0;
Q[39][4] =0.0;
Q[39][5] = 0.0;
Q[39][6] = 0.01;
Q[40][0] = -21.44594383239746;
Q[40][1] =-20.235382080078125;
Q[40][2] =-26.137907028198242;
Q[40][3] =0.0;
Q[40][4] =0.0;
Q[40][5] = 0.0;
Q[40][6] = 0.01;
*/
//	std::vector<std::vector<std::string>> data;
/*	std::ifstream inFile("model.txt");
	while(inFile)
	{
	  std::string s;
	  if (!getline(inFile, s))break;
	  std::istringstream ss(s);
	  std::vector<std::string> record;
	while(ss)
  	{
 	std::string s;
	if (!getline(ss, s, ','))break;
	record.push_back(s);
	std::cout<<record<<std::endl;
	}*/
//	data.push_back(s);
	
//	data.push_back(record);
//	std::cout<<data<<std::endl;



/*
-5.853112697601318,0.0,0.0,0.0,0.0,0.0,0.02
-4.1054511070251465,-11.063689231872559,-17.135744094848633,0.0,0.0,0.0,0.03
-3.8260560035705566,-8.7681884765625,0.0,0.0,0.0,0.0,0.04
-4.455472469329834,0.0,0.0,0.0,0.0,0.0,0.05
-1.751182198524475,0.0,0.0,0.0,0.0,0.0,0.06
-0.6407613754272461,-4.660606384277344,0.0,0.0,0.0,0.0,0.07
0.0,0.0,0.0,0.0,0.0,0.0,0.08
0.0,0.0,0.0,0.0,0.0,0.0,0.09
0.0,0.0,0.0,0.0,0.0,0.0,0.010
0.0,0.0,0.0,0.0,0.0,0.0,0.011
0.0,0.0,0.0,0.0,0.0,0.0,0.012
-61.21909713745117,-190.49937438964844,-167.86428833007812,-177.52488708496094,-226.46038818359375,-295.4471130371094,-260.69262695312513
-3.185408353805542,-11.015246391296387,-22.839447021484375,-28.93031120300293,0.0,0.0,0.014
-4.554518699645996,-7.537059307098389,-19.715612411499023,-23.209415435791016,-32.83228302001953,-57.05420684814453,-70.0850524902343815
-1.5716251134872437,-7.084004878997803,-6.528062343597412,0.0,0.0,0.0,0.016
-1.7260303497314453,-3.346790313720703,-6.602163791656494,0.0,0.0,0.0,0.017
0.0,0.0,0.0,-8.97390079498291,0.0,0.0,0.018
10.86133098602295,0.0,0.0,0.0,-10.018472671508789,0.0,0.019
11.297065734863281,0.0,0.0,0.0,0.0,0.0,0.020
8.587361335754395,0.0,0.0,0.0,0.0,0.0,0.021
0.0,0.0,0.0,0.0,0.0,0.0,0.022
45.13866424560547,0.0,2.034748077392578,0.0,0.0,0.0,0.023
155.33544921875,0.0,0.0,0.0,0.0,0.0,0.024
-50.85114669799805,-162.73110961914062,-146.119384765625,-153.82398986816406,-262.02374267578125,-262.709716796875,-276.604858398437525
-0.2946132719516754,-6.765824317932129,-33.05549621582031,-27.86386489868164,-44.14947509765625,-44.199790954589844,0.026
-0.8693950772285461,-10.704208374023438,-10.018877983093262,-25.813995361328125,-29.407007217407227,-40.15085983276367,-69.0890655517578127
4.0887298583984375,-3.0732152462005615,-5.8245673179626465,-19.98663902282715,-18.68724250793457,-29.144906997680664,-56.6808662414550828
-0.6102585792541504,0.7943301200866699,0.0,0.0,0.0,0.0,-48.19924926757812529
13.79757308959961,0.0,0.0,0.0,0.0,0.0,0.030
16.576576232910156,0.0,0.0,0.0,-4.808135986328125,0.0,-55.4681663513183631
38.58346176147461,0.0,0.0,0.0,0.0,0.0,0.032
40.82941436767578,0.0,0.0,0.0,0.0,0.0,0.033
41.27796936035156,0.0,0.0,0.0,0.0,0.0,0.034
84.28895568847656,0.0,0.0,0.0,0.0,0.0,0.035
183.18910217285156,0.0,0.0,0.0,37.14716720581055,0.0,0.036
-5.259785175323486,-118.54691314697266,-135.1182861328125,-102.07975769042969,-79.79750061035156,-170.06768798828125,-182.1008911132812537
3.812268018722534,0.0,0.0,0.0,-44.1959342956543,0.0,-103.7826995849609438
8.805038452148438,0.0,0.0,-13.435663223266602,0.0,0.0,-125.595680236816439
10.578741073608398,0.0,0.0,0.0,0.0,-45.344661712646484,-42.0788383483886740
8.631324768066406,3.3814587593078613,0.0,0.0,-9.500201225280762,0.0,0.041
14.187300682067871,-0.024870727211236954,0.0,-3.1943557262420654,0.0,0.0,-57.51204681396484442
27.17975425720215,0.0,0.0,0.0,0.0,0.0,0.043
33.90341567993164,0.0,0.0,0.0,0.0,0.0,0.044
43.58867263793945,0.0,0.0,2.070133686065674,0.0,0.0,0.045
59.27653884887695,0.0,0.0,0.0,0.0,0.0,-9.51288414001464846
73.96420288085938,0.0,0.0,0.0,0.0,0.0,0.047
-12.669944763183594,0.0,0.0,52.455623626708984,0.0,49.878326416015625,0.048
-4.924151420593262,-55.29195022583008,-47.86548614501953,-59.981204986572266,-95.649169921875,-93.23382568359375,-201.172164916992249
-0.07682687789201736,-6.334690570831299,-19.357322692871094,-18.580833435058594,-21.93170738220215,-56.403656005859375,-67.0561218261718850
1.3486924171447754,-2.169769763946533,-4.5007710456848145,-13.921704292297363,-18.729276657104492,-56.978885650634766,-82.6572570800781251
-2.507474422454834,-1.5638666152954102,-3.3419106006622314,-12.616859436035156,-22.528549194335938,-22.571699142456055,-39.72674179077148452
-9.921101570129395,-0.20724591612815857,-1.2144502401351929,-7.4093523025512695,-15.102294921875,-31.0244140625,-83.5358428955078153
-13.876439094543457,0.9617952108383179,-1.2330576181411743,-4.882753372192383,-10.660613059997559,-30.487701416015625,-71.906394958496154
-2.4009475708007812,-2.00626277923584,0.00299995020031929,-1.2134019136428833,-8.041404724121094,-17.360763549804688,-54.4533615112304755
-32.43628692626953,-3.911228656768799,-1.024300217628479,0.10935039818286896,-3.2332653999328613,-9.889431953430176,-45.377674102783256
-32.68658447265625,-6.478549480438232,-2.571185827255249,0.15532515943050385,-0.56907719373703,-1.494822382926941,-9.90374660491943457
-43.71143341064453,-3.250215530395508,-3.479182720184326,2.886815071105957,-0.0073937177658081055,0.0,0.058
-18.683700561523438,-14.516260147094727,-8.428376197814941,-0.8202407360076904,1.7712066173553467,0.0,-4.54690456390380959
-38.73809051513672,-53.6545524597168,-54.17335891723633,-28.970504760742188,-21.374374389648438,-14.791523933410645,-2.818469047546386760
0.548691987991333,-8.234733581542969,-49.22521209716797,-44.653255462646484,-39.907772064208984,-83.56261444091797,-97.6570358276367261
-0.055935513228178024,-3.993212938308716,-3.5050435066223145,0.0,0.0,0.0,0.062
-0.9425177574157715,2.875925302505493,0.0,0.0,-16.528480529785156,0.0,0.063
-2.7283360958099365,0.964095413684845,-1.7000705003738403,-4.988280296325684,-15.726778984069824,-18.404878616333008,-37.51120758056640664
-8.296831130981445,-0.20113565027713776,-1.1137372255325317,-3.3542983531951904,-7.007461071014404,-6.77028751373291,-56.58700561523437565
-19.784561157226562,-1.9243454933166504,-0.1349887251853943,-1.7091810703277588,-7.963638782501221,-20.877151489257812,-55.75485229492187566
-30.366113662719727,-4.984386444091797,-1.2498263120651245,-0.41057097911834717,-1.5764830112457275,-14.08419132232666,-42.35489654541015667
-36.036346435546875,-5.3698625564575195,-2.624119281768799,-0.061549242585897446,-1.0940866470336914,-7.253894805908203,-27.04306030273437568
-25.408843994140625,-12.177602767944336,-2.865079879760742,-0.7399885654449463,0.46292880177497864,-1.8853763341903687,-12.47929191589355569
-31.592954635620117,-13.76391315460205,-7.041262149810791,-1.3044350147247314,-0.9075120091438293,0.8288227319717407,-11.59074211120605570
-30.84317398071289,-24.925195693969727,-10.696684837341309,-5.612776756286621,-0.7918370366096497,-0.14640377461910248,-3.07786178588867271
-62.28110885620117,-42.13056182861328,-67.41002655029297,-12.895430564880371,-15.94414234161377,-3.508942127227783,-7.01622819900512772
7.632170677185059,0.0,0.0,0.0,0.0,72.4533920288086,0.073
-0.6915001273155212,0.0,0.0,20.9946231842041,0.0,0.0,0.074
-1.3505234718322754,0.0,0.0,0.0,0.0,0.0,0.075
-2.17258620262146,0.0,0.0,0.0,0.0,32.402626037597656,0.076
-5.387927532196045,-0.30811190605163574,0.3145984411239624,0.0,0.0,0.0,0.077
-8.235013961791992,-1.5441159009933472,-0.5174020528793335,0.8580123782157898,0.0,0.0,39.02929687578
-15.503046989440918,-4.739553451538086,-0.567303478717804,-0.3341214060783386,7.750429630279541,0.0,0.079
-31.726417541503906,-6.408849239349365,-1.7058418989181519,0.5856333374977112,0.0,0.0,0.080
-23.519380569458008,-9.452683448791504,0.0,0.0,0.0,0.0,0.081
-35.09299850463867,-13.177611351013184,-7.872611999511719,0.0,-0.0817311555147171,0.0,0.082
-37.61506652832031,-18.142255783081055,-13.384284019470215,-3.9112255573272705,-0.8665289282798767,0.0,0.083
-100.94767761230469,-87.23189544677734,-47.50800704956055,-9.699459075927734,-15.335384368896484,-14.576886177062988,1.168037176132202184
10.1405029296875,0.0,0.0,0.0,0.0,0.0,0.085
0.0,0.0,0.0,0.0,0.0,27.409961700439453,0.086
-1.7989284992218018,0.0,0.0,8.329419136047363,0.0,0.0,79.2538375854492287
-4.342317581176758,-1.618941307067871,1.0979746580123901,0.0,0.0,0.0,0.088
-8.6430025100708,0.0,0.0,0.0,19.919588088989258,0.0,0.089
-8.428984642028809,-4.507232189178467,-0.6790661215782166,0.0,1.5788847208023071,0.0,0.090
-17.02021598815918,-4.330813884735107,-2.0318973064422607,-1.5664758682250977,4.5057220458984375,0.0,0.091
0.0,0.0,-3.8470864295959473,0.0,0.0,2.2809202671051025,0.092
-44.055206298828125,-14.94748592376709,-8.187196731567383,-2.2107937335968018,-0.9717404246330261,0.0,0.093
-31.873306274414062,-17.880081176757812,-8.631871223449707,-3.092665672302246,-2.3419740200042725,-0.18717314302921295,0.094
-45.607845306396484,-31.79564666748047,-13.70422649383545,-6.819026470184326,-4.140813827514648,-0.6825271844863892,0.095
-148.9290008544922,-207.0484619140625,-103.78492736816406,-47.05143737792969,-33.328514099121094,-34.33827209472656,-0.4172716140747070396
0.02070312574505806,0.0,0.0,0.0,0.0,0.0,0.097
0.0,0.0,0.0,0.0,0.0,0.0,0.098
0.0,0.0,0.0,0.0,0.0,0.0,0.099
0.0,0.0,0.0,0.0,0.0,0.0,0.0100
0.0,0.0,0.0,0.0,0.0,0.0,0.0101
-14.303458213806152,0.0,0.0,0.0,0.0,0.0,25.461946487426758102
-20.58053207397461,-8.673843383789062,0.0,0.0,0.0,0.0,0.0103
-45.16973114013672,-12.60886287689209,0.0,0.0,0.0,0.0,0.0104
0.0,-13.405491828918457,0.0,0.0,0.0,-2.5901072025299072,0.0105
0.0,0.0,0.0,0.0,0.0,-2.233468770980835,0.0106
-56.70635986328125,-24.57134437561035,-23.109010696411133,-12.053750991821289,0.0,0.0,0.0107
-194.7244110107422,-59.56039047241211,-63.386810302734375,-53.9632682800293,-42.96916198730469,-18.313060760498047,-6.15955924987793108
0.0,0.0,0.0,0.0,0.0,0.0,0.0109
0.0,0.0,0.0,0.0,0.0,0.0,0.0110
0.0,0.0,0.0,0.0,0.0,0.0,0.0111
0.0,0.0,0.0,0.0,0.0,0.0,0.0112
0.0,0.0,0.0,0.0,0.0,0.0,0.0113
0.0,0.0,0.0,0.0,0.0,0.0,0.0114
0.0,0.0,0.0,0.0,0.0,0.0,0.0115
0.0,0.0,0.0,0.0,0.0,0.0,0.0116
0.0,0.0,0.0,0.0,0.0,0.0,0.0117
0.0,0.0,0.0,0.0,0.0,0.0,0.0118
0.0,0.0,0.0,0.0,0.0,0.0,0.0119
0.0,0.0,0.0,0.0,0.0,0.0,-6.209778308868408
*/

/*
}
*/



/*char reinforcement_learning::get_next_state(float pitch, float pitch_dot, char action_idx)
{
  float x_current[6][1] = {
    {0},
    {0},
    {pitch},
    {pitch_dot},
    {-actions[action_idx]},
    {actions[action_idx]}};

  char next_state;
  const char u = 1;
  float A_x_current[6][1];
  float x_next[6][1];
  float next_pitch;
  float next_pitch_dot;
  // A*x_current
  for (int i = 1; i < 6; i++)
  {
    for (int j = 1; j < 6; j++)
      {
        A_x_current[i][0]+= A_model[i][j] * x_current[j][0];
      }
  } 
  //x_current + (A*current + B*u)*dt
  for (int i = 1; i < 6; i ++)
  {
    x_next[i][0] = x_current[i][0] + (A_x_current[i][0] + B[i][0])*RL_DELTA;
  }
 
  next_pitch = x_next[2][0];
  next_pitch_dot = x_next[3][0];
  std::cout<<"next pitch"<<next_pitch<<std::endl;
  std::cout<<"next pd " <<next_pitch_dot<<std::endl;
  msg.next_pitch = next_pitch;
  msg.next_pitch_dot = next_pitch_dot;
  next_state = get_state(next_pitch, next_pitch_dot);  

  return next_state;

}*/

class q_learning: public reinforcement_learning
{
  public:
    q_learning();
    ~q_learning();
    std::vector<float> q_row;
    std::vector<int> max_value_idxs;

    char choose_action(char);
    void take_action(int);
};

q_learning::q_learning()
{
    this->q_row.resize(6);
}

q_learning::~q_learning()
{
}

char q_learning::choose_action(char curr_state)
{
  int random_choice;
  float random_num;
  float max_q;
  int action_choice;

  // generate random number to decide whether to explore or exploit
  random_num = fabs((rand()/(float)(RAND_MAX)));	//random num between 0 and 1
  ROS_INFO("random num: %f", random_num);
  msg.action_choice = random_num;
  
  if (random_num < epsilon)
  {
    //pick randomly
    random_choice = rand()%ACTIONS;
    ROS_INFO("random action choice: %d", random_choice);
    msg.random_action = random_choice;
    return random_choice;
  }
  else
  {
    //pick best
    msg.random_action = 50;
    ROS_INFO("picks best");
    this->q_row = Q[curr_state];

    max_q = *std::max_element(q_row.begin(), q_row.end());
    ROS_INFO("max_q: %f", max_q);    
    for (int i = 0; i < q_row.size(); i++)
      {
        if (max_q == q_row[i])
          {
            max_value_idxs.push_back(i);     
	  }
      }

    if (max_value_idxs.size() == 1)
      {
	action_choice = max_value_idxs[0];
	//std::fill(max_value_idxs.being(), max_value_idxs.end(),0);
        max_value_idxs.clear();
	return action_choice;
      }
	else if (max_value_idxs.size() > 1)
      { 
	ROS_INFO("max_value_dx 0 is: %d", max_value_idxs[0]);
	action_choice = max_value_idxs[0];
	max_value_idxs.clear();
//	ROS_INFO("random choice for repeated bests");
//	ROS_INFO("size: %d", max_value_idxs.size());
//        action_choice = rand()%(max_value_idxs.size());
//	max_value_idxs.clear();
//	ROS_INFO("rnadom choice is: %d", action_choice);
        return action_choice;
      }
  }
}



q_learning controller;


namespace gazebo
{

GazeboRsvBalance::GazeboRsvBalance() {}

GazeboRsvBalance::~GazeboRsvBalance() {}

void GazeboRsvBalance::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{

  this->parent_ = _parent;
  this->sdf_ = _sdf;
  this->gazebo_ros_ = GazeboRosPtr(new GazeboRos(_parent, _sdf, "RsvBalancePlugin"));

  this->gazebo_ros_->isInitialized();

  // Obtain parameters from rosparam
  this->gazebo_ros_->getParameter<bool>(this->publish_odom_tf_, "publishOdomTF", true);
  this->gazebo_ros_->getParameter<bool>(this->publish_wheel_joint_, "publishWheelJointState", true);

  this->gazebo_ros_->getParameter<std::string>(this->command_topic_, "commandTopic", "cmd_vel");
  this->gazebo_ros_->getParameter<std::string>(this->odom_topic_, "odomTopic", "odom");
  this->gazebo_ros_->getParameter<std::string>(this->base_frame_id_, "baseFrameId", "base_link");
  this->gazebo_ros_->getParameter<std::string>(this->odom_frame_id_, "odomFrameId", "odom");

  this->gazebo_ros_->getParameter<bool>(this->publish_state_, "publishState", true);
  this->gazebo_ros_->getParameter<double>(this->update_rate_, "updateRate", 50.0);
  this->gazebo_ros_->getParameter<double>(this->publish_state_rate_, "publishStateRate", 50);
  this->gazebo_ros_->getParameter<double>(this->publish_diagnostics_rate_, "publishDiagnosticsRate", 1);

  std::map<std::string, OdomSource> odom_options;
  odom_options["encoder"] = ENCODER;
  odom_options["world"] = WORLD;
  this->gazebo_ros_->getParameter<OdomSource>(this->odom_source_, "odomSource", odom_options, WORLD);

  this->mode_map_["park"] = PARK;
  this->mode_map_["tractor"] = TRACTOR;
  this->mode_map_["balance"] = BALANCE;
  this->gazebo_ros_->getParameter<Mode>(this->current_mode_, "startMode", this->mode_map_, BALANCE);

  if (!this->sdf_->HasElement("wheelSeparation") || !this->sdf_->HasElement("wheelRadius") )
  {
    ROS_ERROR("RsvBalancePlugin - Missing <wheelSeparation> or <wheelDiameter>, Aborting");
    return;
  }
  else
  {
    this->gazebo_ros_->getParameter<double>(this->wheel_separation_, "wheelSeparation");
    this->gazebo_ros_->getParameter<double>(this->wheel_radius_, "wheelRadius");
  }

  this->joints_.resize(2);
  this->joints_[LEFT] = this->gazebo_ros_->getJoint(this->parent_, "leftJoint", "left_joint");
  this->joints_[RIGHT] = this->gazebo_ros_->getJoint(this->parent_, "rightJoint", "right_joint");

  // Control loop timing
  if (this->update_rate_ > 0.0)
  {
    this->update_period_ = 1.0 / this->update_rate_;	//CHANGED TO 5/10 = 0.5SEC = 2HZ
  }
  else
  {
    ROS_WARN("RsvBalancePlugin - Update rate < 0. Update period set to: 0.1. ");
    this->update_period_ = 0.1;
  }
  this->last_update_time_ = this->parent_->GetWorld()->GetSimTime();
  // Variable that control RL algorithm updates
  //this->rl_update_time = this->parent_->GetWorld()->GetSimTime();
 

  // Command velocity subscriber
  ros::SubscribeOptions so = ros::SubscribeOptions::create<geometry_msgs::Twist>(this->command_topic_, 5,
              boost::bind(&GazeboRsvBalance::cmdVelCallback, this, _1),
              ros::VoidPtr(), &this->queue_);
  this->cmd_vel_subscriber_ = this->gazebo_ros_->node()->subscribe(so);
  ROS_INFO("%s: Subscribed to %s!", this->gazebo_ros_->info(), this->command_topic_.c_str());
  // Tilt equilibrium subscriber
  so = ros::SubscribeOptions::create<std_msgs::Float64>("tilt_equilibrium", 5,
              boost::bind(&GazeboRsvBalance::cmdTiltCallback, this, _1),
              ros::VoidPtr(), &this->queue_);
  this->cmd_tilt_subscriber_ = this->gazebo_ros_->node()->subscribe(so);
  ROS_INFO("%s: Subscribed to %s!", this->gazebo_ros_->info(), "tilt_equilibrium");

  // Odometry publisher
  this->odometry_publisher_ = this->gazebo_ros_->node()->advertise<nav_msgs::Odometry>(this->odom_topic_, 10);
  ROS_INFO("%s: Advertise odom on %s !", this->gazebo_ros_->info(), this->odom_topic_.c_str());
  // State publisher
  this->state_publisher_ = this->gazebo_ros_->node()->advertise<rsv_balance_msgs::State>("state", 10);
  ROS_INFO("%s: Advertise system state on %s !", this->gazebo_ros_->info(), "state");
  // Joint state publisher
  this->joint_state_publisher_ = this->gazebo_ros_->node()->advertise<sensor_msgs::JointState>("joint_states", 10);
  ROS_INFO("%s: Advertise joint_states!", gazebo_ros_->info());

  this->Q_state_publisher_ = this->gazebo_ros_->node()->advertise<rsv_balance_msgs::Q_state>("Q_state", 10);
  


  // Service for changing operating mode
  ros::AdvertiseServiceOptions ao = ros::AdvertiseServiceOptions::create<rsv_balance_msgs::SetMode>("set_mode",
        boost::bind(&GazeboRsvBalance::setMode, this, _1),
        ros::VoidPtr(),
        &this->queue_);
  this->set_mode_server_ = this->gazebo_ros_->node()->advertiseService(ao);
  // Service for chaning input source
  ao = ros::AdvertiseServiceOptions::create<rsv_balance_msgs::SetInput>("set_input",
        boost::bind(&GazeboRsvBalance::setInput, this, _1),
        ros::VoidPtr(),
        &this->queue_);
  this->set_input_server_ = this->gazebo_ros_->node()->advertiseService(ao);
  // Reset override service
  ao = ros::AdvertiseServiceOptions::create<std_srvs::Empty>("reset_override",
        boost::bind(&GazeboRsvBalance::resetOverride, this, _1),
        ros::VoidPtr(),
        &this->queue_);
  this->reset_override_server_ = this->gazebo_ros_->node()->advertiseService(ao);
  // Reset odom service
  ao = ros::AdvertiseServiceOptions::create<std_srvs::Empty>("reset_odom",
        boost::bind(&GazeboRsvBalance::resetOdom, this, _1),
        ros::VoidPtr(),
        &this->queue_);
  this->reset_odom_server_ = this->gazebo_ros_->node()->advertiseService(ao);

  // Broadcaster for TF
  this->transform_broadcaster_ = boost::shared_ptr<tf::TransformBroadcaster>(new tf::TransformBroadcaster());

  this->resetVariables();
  this->state_control_.resetControl();
  this->x_desired_ = 0;
  this->rot_desired_ = 0;
  this->tilt_desired_ = 0;
  this->u_control_ = this->state_control_.getControl();


  this->alive_ = true;
  // start custom queue
  this->callback_queue_thread_ = boost::thread(boost::bind(&GazeboRsvBalance::QueueThread, this));
  // listen to the update event (broadcast every simulation iteration)
 
 this->update_connection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboRsvBalance::UpdateChild, this));

}

/*!
* \brief Resets simulation variables.
* 
* Used when starting and when gazebo reset world or model
*/
void GazeboRsvBalance::resetVariables()
{
  this->x_desired_ = 0;
  this->rot_desired_ = 0;
  this->odom_offset_pos_ = math::Vector3(0, 0, 0);
  this->odom_offset_rot_ = math::Vector3(0, 0, 0);
}

/*!
* \brief Sets platform operating mode.
*/
bool GazeboRsvBalance::setMode(rsv_balance_msgs::SetMode::Request  &req)
{
  // Ugly implementation means bad concept
  switch (req.mode)
  {
    case rsv_balance_msgs::SetModeRequest::PARK:
      ROS_INFO("%s: Mode: park", this->gazebo_ros_->info());
      this->current_mode_ = PARK;
      break;
    case rsv_balance_msgs::SetModeRequest::TRACTOR:
      ROS_INFO("%s: Mode: tractor", this->gazebo_ros_->info());
      this->current_mode_ = TRACTOR;
      break;
    case rsv_balance_msgs::SetModeRequest::BALANCE:
      ROS_INFO("%s: Mode: balance", this->gazebo_ros_->info());
      this->current_mode_ = BALANCE;
      break;
    default:
      return false;
  };
  return true;
}

/*!
* \brief Just exposes service. Not used in simulation
*/
bool GazeboRsvBalance::setInput(rsv_balance_msgs::SetInput::Request  &req)
{
  // In simulation input should always be serial communication.
  ROS_INFO("%s: Input: %d", this->gazebo_ros_->info(), req.input);
  return true;
}

/*!
* \brief Just exposes service. Not used in simulation
*/
bool GazeboRsvBalance::resetOverride(std_srvs::Empty::Request  &req)
{
  // In simulation we don't have RC override, nothing to reset
  ROS_INFO("%s: Reset Override", this->gazebo_ros_->info());
  return true;
}

/*!
* \brief Service to reset odometry.
*/
bool GazeboRsvBalance::resetOdom(std_srvs::Empty::Request  &req)
{
  ROS_INFO("%s: Reset Odom", this->gazebo_ros_->info());
  this->resetOdometry();
  return true;
}

/*!
* \brief Callback to cmd_vel
*/
void GazeboRsvBalance::cmdVelCallback(const geometry_msgs::Twist::ConstPtr& cmd_msg)
{
  this->x_desired_ = cmd_msg->linear.x;
  this->rot_desired_ = cmd_msg->angular.z;
}

/*!
* \brief Callback to cmd_tilt
*/
void GazeboRsvBalance::cmdTiltCallback(const std_msgs::Float64::ConstPtr& cmd_tilt)
{
  this->tilt_desired_ = cmd_tilt->data;
}

/*!
* \brief Gets pitch angle values directly from Gazebo world
*/
void GazeboRsvBalance::updateIMU()
{
  // Store pitch and dpitch
  math::Pose pose = this->parent_->GetWorldPose();
  math::Vector3 veul = this->parent_->GetRelativeAngularVel();
  this->imu_pitch_ = pose.rot.GetPitch();
  this->imu_dpitch_ = veul.y;
}

/*!
* \brief Resets odometry by adding offset to WORLD odometry, and resetting odometry values.
*/
/** @todo Actually implement it */
void GazeboRsvBalance::resetOdometry()
{
  math::Pose pose = this->parent_->GetWorldPose();
  this->odom_offset_pos_ = pose.pos;
  this->odom_offset_rot_.z = pose.rot.GetYaw();
}

/*!
* \brief Updates odometry, from Gazebo world or from encoders.
*/
/** @todo Implement encoder odometry */
void GazeboRsvBalance::updateOdometry()
{
  double ang_velocity_left = -this->joints_[LEFT]->GetVelocity(0);
  double ang_velocity_right = this->joints_[RIGHT]->GetVelocity(0);

  this->feedback_v_ = this->wheel_radius_/2.0 * (ang_velocity_right + ang_velocity_left);
  this->feedback_w_ = this->wheel_radius_/this->wheel_separation_ * (ang_velocity_right - ang_velocity_left);

  if (odom_source_ == WORLD)
  {
    math::Pose pose = this->parent_->GetWorldPose();
    math::Vector3 velocity_linear = this->parent_->GetRelativeLinearVel();
    math::Vector3 velocity_angular = this->parent_->GetRelativeAngularVel();
    // TODO(vmatos): reset odometry by setting an offset in world
    // this->odom_.pose.pose.position.x = pose.pos[0] - this->odom_offset_pos_[0];
    // this->odom_.pose.pose.position.y = pose.pos[1] - this->odom_offset_pos_[1];
    // this->odom_.pose.pose.position.z = pose.pos[2] - this->odom_offset_pos_[2];
    this->odom_.pose.pose.position.x = pose.pos[0];
    this->odom_.pose.pose.position.y = pose.pos[1];
    this->odom_.pose.pose.position.z = pose.pos[2];
    // tf::Quaternion qt;
    // qt.setRPY(pose.rot.GetRoll(), pose.rot.GetPitch(), pose.rot.GetYaw() - this->odom_offset_rot_[2]);
    // qt.setRPY(pose.rot.GetRoll(), pose.rot.GetPitch(), pose.rot.GetYaw());
    this->odom_.pose.pose.orientation.x = pose.rot.x;
    this->odom_.pose.pose.orientation.y = pose.rot.y;
    this->odom_.pose.pose.orientation.z = pose.rot.z;
    this->odom_.pose.pose.orientation.w = pose.rot.w;
    this->odom_.twist.twist.linear.x = velocity_linear[0];
    this->odom_.twist.twist.linear.y = velocity_linear[1];
    this->odom_.twist.twist.angular.z = velocity_angular.z;
  }
  else
  {
    ROS_WARN("%s - Odometry from other sources not yet supported.", this->gazebo_ros_->info());
  }
}

/*!
* \brief Publishes odometry and desired tfs
*/
/** @todo User configurable covariance */
void GazeboRsvBalance::publishOdometry()
{
  ros::Time current_time = ros::Time::now();
  std::string odom_frame = this->gazebo_ros_->resolveTF(this->odom_frame_id_);
  std::string base_frame = this->gazebo_ros_->resolveTF(this->base_frame_id_);

  // set odometry covariance
  this->odom_.pose.covariance[0] = 0.00001;
  this->odom_.pose.covariance[7] = 0.00001;
  this->odom_.pose.covariance[14] = 1000000000000.0;
  this->odom_.pose.covariance[21] = 1000000000000.0;
  this->odom_.pose.covariance[28] = 1000000000000.0;
  this->odom_.pose.covariance[35] = 0.001;
  // set header
  this->odom_.header.stamp = current_time;
  this->odom_.header.frame_id = odom_frame;
  this->odom_.child_frame_id = base_frame;
  //  Publish odometry
  this->odometry_publisher_.publish(this->odom_);

  if (this->publish_odom_tf_)
  {
    tf::Vector3 vt;
    tf::Quaternion qt;
    vt = tf::Vector3(this->odom_.pose.pose.position.x,
                     this->odom_.pose.pose.position.y,
                     this->odom_.pose.pose.position.z);
    qt = tf::Quaternion(this->odom_.pose.pose.orientation.x, this->odom_.pose.pose.orientation.y,
                        this->odom_.pose.pose.orientation.z, this->odom_.pose.pose.orientation.w);
    tf::Transform base_to_odom(qt, vt);
    this->transform_broadcaster_->sendTransform(
            tf::StampedTransform(base_to_odom, current_time, odom_frame, base_frame));
  }
}

/*!
* \brief Publishes wheel joint_states
*/
void GazeboRsvBalance::publishWheelJointState()
{
  ros::Time current_time = ros::Time::now();

  sensor_msgs::JointState joint_state;
  joint_state.header.stamp = current_time;
  joint_state.name.resize(joints_.size());
  joint_state.position.resize(joints_.size());

  for (int i = 0; i < 2; i++)
  {
    physics::JointPtr joint = this->joints_[i];
    math::Angle angle = joint->GetAngle(0);
    joint_state.name[i] = joint->GetName();
    joint_state.position[i] = angle.Radian();
  }
  this->joint_state_publisher_.publish(joint_state);
}

/*!
* \brief Called when Gazebo resets world
*/
void GazeboRsvBalance::Reset()
{
  this->resetVariables();
  this->state_control_.resetControl();
  // Reset control
  this->last_update_time_ = this->parent_->GetWorld()->GetSimTime();
  this->current_mode_ = BALANCE;
  this->imu_pitch_ = 0;
  this->imu_dpitch_ = 0;
  this->feedback_v_ = 0;
  this->feedback_w_ = 0;
}

void GazeboRsvBalance::publishQstate()
{
  rsv_balance_msgs::Q_state q_msg;
  ros::Time current_time = ros::Time::now();
  for (int i = 0; i < ACTIONS; i++)
  {
    q_msg.state0[i] = controller.Q[0][i];
    q_msg.state1[i] = controller.Q[1][i];
    q_msg.state2[i] = controller.Q[2][i];
    q_msg.state3[i] = controller.Q[3][i];
    q_msg.state4[i] = controller.Q[4][i];
    q_msg.state5[i] = controller.Q[5][i];
    q_msg.state6[i] = controller.Q[6][i];
    q_msg.state7[i] = controller.Q[7][i];
    q_msg.state8[i] = controller.Q[8][i];
    q_msg.state9[i] = controller.Q[9][i];
    q_msg.state10[i] = controller.Q[10][i];
    q_msg.state11[i] = controller.Q[11][i];
    q_msg.state12[i] = controller.Q[12][i];
    q_msg.state13[i] = controller.Q[13][i];
    q_msg.state14[i] = controller.Q[14][i];
    q_msg.state15[i] = controller.Q[15][i];
    q_msg.state16[i] = controller.Q[16][i];
    q_msg.state17[i] = controller.Q[17][i];
    q_msg.state18[i] = controller.Q[18][i];
    q_msg.state19[i] = controller.Q[19][i];
    q_msg.state20[i] = controller.Q[20][i];
    q_msg.state21[i] = controller.Q[21][i];
    q_msg.state22[i] = controller.Q[22][i];
    q_msg.state23[i] = controller.Q[23][i];
    q_msg.state24[i] = controller.Q[24][i];
    q_msg.state25[i] = controller.Q[25][i];
    q_msg.state26[i] = controller.Q[26][i];
    q_msg.state27[i] = controller.Q[27][i];
    q_msg.state28[i] = controller.Q[28][i];
    q_msg.state29[i] = controller.Q[29][i];
    q_msg.state30[i] = controller.Q[30][i];
    q_msg.state31[i] = controller.Q[31][i];
    q_msg.state32[i] = controller.Q[32][i];
    q_msg.state33[i] = controller.Q[33][i];
    q_msg.state34[i] = controller.Q[34][i];
    q_msg.state35[i] = controller.Q[35][i];
    q_msg.state36[i] = controller.Q[36][i];
    q_msg.state37[i] = controller.Q[37][i];
    q_msg.state38[i] = controller.Q[38][i];
    q_msg.state39[i] = controller.Q[39][i];
    q_msg.state40[i] = controller.Q[40][i];
    q_msg.state41[i] = controller.Q[41][i];
    q_msg.state42[i] = controller.Q[42][i];
    q_msg.state43[i] = controller.Q[43][i];
    q_msg.state44[i] = controller.Q[44][i];
    q_msg.state45[i] = controller.Q[45][i];
    q_msg.state46[i] = controller.Q[46][i];
    q_msg.state47[i] = controller.Q[47][i];
    q_msg.state48[i] = controller.Q[48][i];
    q_msg.state49[i] = controller.Q[49][i];
    q_msg.state50[i] = controller.Q[50][i];
    q_msg.state51[i] = controller.Q[51][i];
    q_msg.state52[i] = controller.Q[52][i];
    q_msg.state53[i] = controller.Q[53][i];
    q_msg.state54[i] = controller.Q[54][i];
    q_msg.state55[i] = controller.Q[55][i];
    q_msg.state56[i] = controller.Q[56][i];
    q_msg.state57[i] = controller.Q[57][i];
    q_msg.state58[i] = controller.Q[58][i];
    q_msg.state59[i] = controller.Q[59][i];
    q_msg.state60[i] = controller.Q[60][i];
    q_msg.state61[i] = controller.Q[61][i];
    q_msg.state62[i] = controller.Q[62][i];
    q_msg.state63[i] = controller.Q[63][i];
    q_msg.state64[i] = controller.Q[64][i];
    q_msg.state65[i] = controller.Q[65][i];
    q_msg.state66[i] = controller.Q[66][i];
    q_msg.state67[i] = controller.Q[67][i];
    q_msg.state68[i] = controller.Q[68][i];
    q_msg.state69[i] = controller.Q[69][i];
    q_msg.state70[i] = controller.Q[70][i];
    q_msg.state71[i] = controller.Q[71][i];
    q_msg.state72[i] = controller.Q[72][i];
    q_msg.state73[i] = controller.Q[73][i];
    q_msg.state74[i] = controller.Q[74][i];
    q_msg.state75[i] = controller.Q[75][i];
    q_msg.state76[i] = controller.Q[76][i];
    q_msg.state77[i] = controller.Q[77][i];
    q_msg.state78[i] = controller.Q[78][i];
    q_msg.state79[i] = controller.Q[79][i];
    q_msg.state80[i] = controller.Q[80][i];
    q_msg.state81[i] = controller.Q[81][i];
    q_msg.state82[i] = controller.Q[82][i];
    q_msg.state83[i] = controller.Q[83][i];
    q_msg.state84[i] = controller.Q[84][i];
    q_msg.state85[i] = controller.Q[85][i];
    q_msg.state86[i] = controller.Q[86][i];
    q_msg.state87[i] = controller.Q[87][i];
    q_msg.state88[i] = controller.Q[88][i];
    q_msg.state89[i] = controller.Q[89][i];
    q_msg.state90[i] = controller.Q[90][i];
    q_msg.state91[i] = controller.Q[91][i];
    q_msg.state92[i] = controller.Q[92][i];
    q_msg.state93[i] = controller.Q[93][i];
    q_msg.state94[i] = controller.Q[94][i];
    q_msg.state95[i] = controller.Q[95][i];
    q_msg.state96[i] = controller.Q[96][i];
    q_msg.state97[i] = controller.Q[97][i];
    q_msg.state98[i] = controller.Q[98][i];
    q_msg.state99[i] = controller.Q[99][i];
    q_msg.state100[i] = controller.Q[100][i];
    q_msg.state101[i] = controller.Q[101][i];
    q_msg.state102[i] = controller.Q[102][i];
    q_msg.state103[i] = controller.Q[103][i];
    q_msg.state104[i] = controller.Q[104][i];
    q_msg.state105[i] = controller.Q[105][i];
    q_msg.state106[i] = controller.Q[106][i];
    q_msg.state107[i] = controller.Q[107][i];
    q_msg.state108[i] = controller.Q[108][i];
    q_msg.state109[i] = controller.Q[109][i];
    q_msg.state110[i] = controller.Q[110][i];
    q_msg.state111[i] = controller.Q[111][i];
    q_msg.state112[i] = controller.Q[112][i];
    q_msg.state113[i] = controller.Q[113][i];
    q_msg.state114[i] = controller.Q[114][i];
    q_msg.state115[i] = controller.Q[115][i];
    q_msg.state116[i] = controller.Q[116][i];
    q_msg.state117[i] = controller.Q[117][i];
    q_msg.state118[i] = controller.Q[118][i];
    q_msg.state119[i] = controller.Q[119][i];


 } 
// publish important data
  this->Q_state_publisher_.publish(q_msg);

}



/*!
* \brief Gazebo step update
*/
void GazeboRsvBalance::UpdateChild()
{
  common::Time current_time = this->parent_->GetWorld()->GetSimTime();
  double seconds_since_last_update = (current_time - this->last_update_time_).Double();
  char action_idx;  
  float pitch;
  char current_state;
  char next_state;
  char state;
  char action_taken;
  char action_idx_tmp;
  float reward;
  float next_pitch;
  float next_pitch_dot;

  this->updateIMU();
  this->updateOdometry();
  this->publishOdometry();
  this->publishWheelJointState();


  pitch = this->imu_pitch_*(180/M_PI);
  if (std::abs(pitch) > PITCH_THRESHOLD)
    {
      this->restart_delta = parent_->GetWorld()->GetSimTime();
      if (this->restart_delta - this->restart_delta_prev > 0.1)
	{

	  ROS_INFO("RESTART SIM - pitch is: %f!", this->imu_pitch_*(180/M_PI));
	  controller.episode_num++;
          controller.msg.episodes = controller.episode_num;
	  
	  //initalise appropriate variables
	  controller.time_steps = 0;
	  controller.prev_pitch = 0;
	  controller.pitch_dot = 0;
	  controller.reward_per_ep = 0;

	  //clear message
	  controller.msg.pitch = 0;
	  controller.msg.pitch_dot = 0;
	  controller.msg.action = 0;
	  controller.msg.action_idx = 0;
	  controller.msg.current_state = 0;
	  controller.msg.next_state = 0;
	  controller.msg.td_update = 0;
	  controller.msg.td_target = 0;
	  controller.msg.td_error = 0;
	}
	this->restart_delta_prev = this->restart_delta;
	
      }


  // Only execute control loop on specified rate
  if (seconds_since_last_update > RL_DELTA)
  {

    //ROS_INFO("seconds since last update %f", seconds_since_last_update);

    this->last_update_time_ += common::Time(RL_DELTA);

   switch (this->current_mode_)
    {
     case BALANCE:

      // apply control if segway is still in pitch range
      if (std::abs(pitch) <= PITCH_THRESHOLD)
      {
	// class member variables
	controller.pitch = pitch;
	controller.pitch_dot = (controller.pitch - controller.prev_pitch)/RL_DELTA;

	// message member variables
	controller.msg.pitch = controller.pitch;
	controller.msg.pitch_dot = controller.pitch_dot;
	controller.msg.epsilon = controller.epsilon;
	controller.msg.time_steps = controller.time_steps;
	controller.msg.error = controller.pitch - REFERENCE_PITCH;
	controller.msg.prev_pitch = controller.prev_pitch;
	
	// get state of the system
	state = controller.get_state(controller.pitch, controller.pitch_dot);

	// debugging data:	
	ROS_INFO("episode: %d", controller.episode_num);
	ROS_INFO("time step: %d", controller.time_steps);	
	ROS_INFO("pitch: %f", controller.pitch);
	ROS_INFO("pitch prev: %f", controller.prev_pitch);
 	ROS_INFO("pitch dot: %f", controller.pitch_dot);
	ROS_INFO("state: %d", state);

	// first iteration
	if (controller.time_steps < 1)
	{
   	  // set the current state
  	  controller.current_state = state;
	  ROS_INFO("current state: %d", controller.current_state);

    	  // select action
	  controller.action_idx = controller.choose_action(controller.current_state);
	  controller.action = actions[controller.action_idx];
	  ROS_INFO("action idx %d and action: %d", controller.action_idx, controller.action);	
	 
	  //take action
	  this->joints_[LEFT]->SetForce(0,-controller.action);
	  this->joints_[RIGHT]->SetForce(0, controller.action);


	}else if (controller.time_steps >= 1)
	{
   	  // set the  next state 
	  controller.next_state = state;
	  controller.msg.next_state = controller.next_state;
	  ROS_INFO("next state: %d", controller.next_state);
	  
	  // get reward
	  reward = controller.get_reward(controller.next_state);
	  controller.msg.reward = reward;
	  ROS_INFO("reward is %f", reward);	

	  controller.reward_per_ep+=reward;
	  controller.msg.reward_per_ep = controller.reward_per_ep;

	  //TD update
	  controller.TD_update(controller.current_state, controller.action_idx, controller.next_state, reward);
	  
	  // publish Q(s,a) matrix
	  this->publishQstate();	
	  // publish state data
  	  this->state_publisher_.publish(controller.msg);
	
	  // Now move from the next state to the current state
    	  controller.current_state = controller.next_state;
 
	  // select action
	  controller.action_idx = controller.choose_action(controller.current_state);
	  controller.action = actions[controller.action_idx];
	  ROS_INFO("action idx %d and action: %d", controller.action_idx, controller.action);	
	  
	  // take action
	  this->joints_[LEFT]->SetForce(0,-controller.action);
	  this->joints_[RIGHT]->SetForce(0, controller.action);

 	}
 
	// set prev pitch
	controller.prev_pitch = controller.pitch;

	// add data to message (state, action, action_idx,  to msg
	controller.msg.current_state = controller.current_state;
	controller.msg.action = controller.action;
	controller.msg.action_idx = controller.action_idx;

	//increment timestep
	controller.time_steps++;

	//decrease epsilon every 20 eps
	if (controller.episode_num % 15 == 0 && controller.episode_num > 1)
	{
	  this->epsilon_delta = parent_->GetWorld()->GetSimTime();
	  if (this->epsilon_delta - this->epsilon_delta_prev > 0.1)
	  {
	    ROS_INFO("DECREASE PARAMS");
	    controller.epsilon = controller.epsilon/2;
	    controller.msg.epsilon = controller.epsilon;
	  }
	  this->epsilon_delta_prev = epsilon_delta;
	}

	//end simulation
	if (controller.episode_num == MAX_EPISODE)
	{
	  ROS_INFO("SIMULATION COMPLETE AT %d EPISODES", controller.episode_num);
	  while(1){}
	}

	//add disturbances
	this->disturbance_time = parent_->GetWorld()->GetSimTime();
	if (this->disturbance_time - this->disturbance_time_prev > 1)
	{
	   if (controller.disturbance_cntr % 2 == 0)
	   {
	     this->joints_[LEFT]->SetForce(0,-100);
	     this->joints_[RIGHT]->SetForce(0, 100);
	   }else{
	     this->joints_[LEFT]->SetForce(0,100);
	     this->joints_[RIGHT]->SetForce(0, -100);
	   }
	   controller.disturbance_cntr++;
	   controller.msg.disturbance = controller.disturbance_cntr;
	   this->disturbance_time_prev = this->disturbance_time;
	   ROS_INFO("DISTURBANCE!!!");
	}


      }	


	
      break;
    case TRACTOR:
      this->joints_[LEFT]->SetVelocity(0, -( (2.0*this->x_desired_ - this->rot_desired_*this->wheel_separation_)
                                                                                        / (this->wheel_radius_*2.0)));
      this->joints_[RIGHT]->SetVelocity(0, ( (2.0*this->x_desired_ + this->rot_desired_*this->wheel_separation_)
                                                                                        / (this->wheel_radius_*2.0)));
      break;
    case PARK:
      this->joints_[LEFT]->SetVelocity(0, 0);
      this->joints_[RIGHT]->SetVelocity(0, 0);
      this->joints_[LEFT]->SetForce(0, 0);
      this->joints_[RIGHT]->SetForce(0, 0);
      break;
  };
}	//end of if
}

/*!
* \brief Called by gazebo upon exiting
*/
void GazeboRsvBalance::FiniChild()
{
  this->alive_ = false;
  this->queue_.clear();
  this->queue_.disable();
  this->gazebo_ros_->node()->shutdown();
  this->callback_queue_thread_.join();
}

void GazeboRsvBalance::QueueThread()
{
  static const double timeout = 0.01;
  while (this->alive_ && this->gazebo_ros_->node()->ok())
  {
    this->queue_.callAvailable(ros::WallDuration(timeout));
  }
}

GZ_REGISTER_MODEL_PLUGIN(GazeboRsvBalance)
}  // namespace gazebo

