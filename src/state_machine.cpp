#include "ros/ros.h"
#include "rt2_assignment1/Command.h"
#include "rt2_assignment1/RandomPosition.h"
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <rt2_assignment1/GoalReachingAction.h>

bool start = false;

bool user_interface(rt2_assignment1::Command::Request &req, rt2_assignment1::Command::Response &res){
    if (req.command == "start"){
    	start = true;
    }
    else {
    	start = false;
    }
    return true;
}


int main(int argc, char **argv)
{
   ros::init(argc, argv, "state_machine");
   ros::NodeHandle n;
   ros::ServiceServer service= n.advertiseService("/user_interface", user_interface);
   ros::ServiceClient client_rp = n.serviceClient<rt2_assignment1::RandomPosition>("/position_server");
   // create the action client
   // true causes the client to spin its own thread
   actionlib::SimpleActionClient<rt2_assignment1::GoalReachingAction> ac("go_to_point", true);
   rt2_assignment1::RandomPosition rp;
   // send a goal to the action
   rt2_assignment1::GoalReachingGoal goal;
   
   rp.request.x_max = 5.0;
   rp.request.x_min = -5.0;
   rp.request.y_max = 5.0;
   rp.request.y_min = -5.0;
 
   
   while(ros::ok()){
   	ros::spinOnce();
   	if (start){
   		client_rp.call(rp);
   		ROS_INFO("Waiting for action server to start.");
  		// wait for the action server to start
  		ac.waitForServer(); // will wait for infinite time
  		goal.x = rp.response.x;
  		goal.y = rp.response.y;
  		goal.theta = rp.response.theta;
  		std::cout << "\nGoing to the position: x= " << goal.x << " y= " << goal.y << " theta = " <<goal.theta << std::endl;
  		ac.sendGoal(goal);
  		
		// wait for the action to return
 		bool finished_before_timeout = ac.waitForResult(ros::Duration(120.0));
		
		if (finished_before_timeout)
		{
		   ROS_INFO("Postion reacheded ");
		}
		else
		   ROS_INFO("Action did not finish before the time out.");

  //exit
 
   		
   		
   		std::cout << "Position reached" << std::endl;
   		
   	}
   }
   return 0;
}
