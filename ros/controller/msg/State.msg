# State space vector for balance system

# rsv:
float64 theta
float64 dx
float64 dphi
float64 dtheta

#customized for PID:
float64 error
float64 cmd
float64 error_proportional
float64 error_integral
float64 integral_sum
float64 error_derivative
float64 derivative
int64 episodes
int64 time_steps

# customized for RL:
int32 wins
int32 loses
float64 prev_pitch

float64 pitch
float64 pitch_dot
int16 action
float64 reward
float64 reward_per_ep
char current_state
char next_state
float64 td_update
float64 td_target
float64 td_error
int16 max_action_idx
char action_idx
float64 alpha
float64 discount_factor
float64 td_update_test
float64 epsilon
float64 action_choice
int16 random_action

