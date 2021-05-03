#include <inttypes.h>
#include <memory>
#include "rt2_assignment1/srv/position_server.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"

using RandomPosition = rt2_assignment1::srv::RandomPosition;
using std::placeholders::_1; // they will be replaced by the actual message during the run
using std::placeholders::_2;
using std::placeholders::_3;

namespace rt2_assignment1{
class MinimalServer : public rclcpp::Node
{
public:
  MinimalServer(const rclcpp::NodeOptions & options)
  : Node("minimal_server", options)
  {
    service_ = this->create_service<RandomPosition>(
      "position_server", std::bind(&MinimalServer::handle_service, this, _1, _2, _3)); // callback
  }

private:

  void handle_service(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<RandomPosition::Request> req,
  const std::shared_ptr<RandomPosition::Response> res)
  {
  (void)request_header;
    res->x = req->x_min + (rand() / ( RAND_MAX / (req->x_max-req->x_min) ) );
    res->y = req->y_min + (rand() / ( RAND_MAX / (req->y_max-req->y_min) ) );
    res->theta =  -3.14+ (rand() / ( RAND_MAX / (6.28)))
  }
  rclcpp::Service<RandomPosition>::SharedPtr service_; // service var: pointer to rclcpp service
  
  
};
}
RCLCPP_COMPONENTS_REGISTER_NODE(rt2_assignment1::MinimalServer)
