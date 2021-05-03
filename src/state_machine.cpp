#include <inttypes.h>
#include <memory>
#include "rt2_assignment1/srv/position_server.hpp"
#include "rt2_assignment1/srv/go_to_point.hpp"
#include "rt2_assignment1/srv/user_interface.hpp"
#include "rclcpp_components/register_node_macro.hpp"


#include "rclcpp/rclcpp.hpp"

using RandomPosition = rt2_assignment1::srv::RandomPosition;
using std::placeholders::_1; // they will be replaced by the actual message during the run
using std::placeholders::_2;
using std::placeholders::_3;

namespace my_composition_package{
class FSM : public rclcpp::Node
{
public:
  FSM(const rclcpp::NodeOptions & options)
  : Node("minimal_server", options)
  {
    service_ = this->create_service<Command>(
      "user_interface", std::bind(&FSM::handle_service, this, _1, _2, _3)); // callback
    client_1 = this->create_client<Position>("go_to_point");
    while (!client_1->wait_for_service(std::chrono::seconds(1))){
     if (!rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(), "client_1 interrupted while waiting for service to appear.");
      return;
    }}
    RCLCPP_INFO(this->get_logger(), "waiting for service to appear...");
    client_2 = this->create_client<RandomPosition>("position_server"); 
    while (!client_2->wait_for_service(std::chrono::seconds(1))){
     if (!rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(), "client_2 interrupted while waiting for service to appear.");
      return;
    }}

    RCLCPP_INFO(this->get_logger(), "waiting for service to appear..."); 

  this->request_1 = std::make_shared<Position::Request>();
  this->response_1 = std::make_shared<Position::Response>();
  this->request_2 = std::make_shared<RandomPosition::Request>();
  this->response_2 = std::make_shared<RandomPosition::Response>();
  state_mach();
  }

  

private:
  void state_mach(){
  
   this->request_2->x_max = 5.0;
   this->request_2->x_min = -5.0;
   this->request_2->y_max = 5.0;
   this->request_2->y_min = -5.0;
  
  
   while(ros::ok()){
   	ros::spinOnce();
   	if (start){
   		client_2.call(this->request_2);
   		this->request_1->x = this->response_2->x;
   		this->request_1->y = this->response_2->y;
   		this->request_1->theta = this->response_2->theta;
   		std::cout << "\nGoing to the position: x= " << this->request_1->x << " y= " <<this->request_1->y << " theta = " <<this->request_1->theta << std::endl;
   		client_1.call(this->request_1);
   		std::cout << "Position reached" << std::endl;
   	}
   }
  }

  void handle_service(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<RandomPosition::Request> req,
  const std::shared_ptr<RandomPosition::Response> res)
  {
  (void)request_header;
     if (req->command == "start"){
    	start = true;
    }
    else {
    	start = false;
    }
}
  start = false;
  rclcpp::Service<Command>::SharedPtr service_; // service var: pointer to rclcpp service
  rclcpp::Client<Position>::SharedPtr client_1; // service var: pointer to rclcpp service
  rclcpp::Client<RandomPosition>::SharedPtr client_2; // service var: pointer to rclcpp service
  
  
};
}
RCLCPP_COMPONENTS_REGISTER_NODE(manual_composition_package::MinimalSubscriber)