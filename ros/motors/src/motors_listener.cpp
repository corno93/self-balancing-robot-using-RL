

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <stdio.h>
#include <wiringSerial.h>
#include <wiringPi.h>

class Motors
{
		//Replace type of message with message containing the encoder values
		void encoder_callback(const std_msgs::String::ConstPtr& msg);
	public:
		//Ros Node Handle
  		ros::NodeHandle n;
		
		//Motor Encoders
		int m_left_encoder_value;
		int m_right_encoder_value;

		//Figure out what type is fd
		some_type fd;
};

void Motors::encoder_callback(const std_msgs::String::ConstPtr& msg)
{
	//Need to include the encoder value in the message received
	//m_right_encoder_value = msg.encoder_value;
	//m_left_encoder_value = msg.encoder_value;
	
	ROS_INFO("I heard: [%s]", msg->data.c_str());
	//Do not publish the serial in the callback!!
	//serialPutChar(fd, int(msg->data.c_str()));
}


void Motors::init()
{
	//Ros Init
	ros::Subscriber sub = n.subscribe("encoder_topic", 1000, this->ecoder_callback);

	//Serial Init
/*	if ((fd = serialOpen("/dev/ttyS0",9600))<0)
	{
		ROS_INFO("Unable to open serial device");
	}	
	if (wiringPiSetup() == -1)
	{
		ROS_INFO("Unable to start wiringPi");
	}
*/
}

void Motors::write_serial_command()
{
	//Here you write to the serial
}

int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "motors_listener");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  //TODO:Check where node handle should live, Roman thinks that it is in the class

  /**
   * The subscribe() call is how you tell ROS that you want to receive messages
   * on a given topic.  This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing.  Messages are passed to a callback function, here
   * called chatterCallback.  subscribe() returns a Subscriber object that you
   * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
   * object go out of scope, this callback will automatically be unsubscribed from
   * this topic.
   *
   * The second parameter to the subscribe() function is the size of the message
   * queue.  If messages are arriving faster than they are being processed, this
   * is the number of messages that will be buffered up before beginning to throw
   * away the oldest ones.
   */

  //TODO: Determine frequency that it writes.
  ros::Rate loop_rate(10);


  Motors motors;
  motors.init();
  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
  int count = 0;
  while (ros::ok())
  {
	motors.step();
	motors.send_serial_command();
    	ROS_INFO("%s", msg.data.c_str());

	ros::spinOnce();

	loop_rate.sleep();
	++count;
  }
  return 0;
}


