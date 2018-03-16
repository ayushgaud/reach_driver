#include <cstdio>
#include "geometry_msgs/TwistWithCovarianceStamped.h"
#include "reach_parser.h"
#include "ros/ros.h"
#include "sensor_msgs/NavSatFix.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "gps");
  ros::NodeHandle n("~");

  GPS_State states;

  std::string port;
  n.param<std::string>("port", port, "auto");

  GPS_ERB gps_object(states, port.c_str());

  ros::Publisher gps_pub = n.advertise<sensor_msgs::NavSatFix>("fix", 100);
  ros::Publisher gps_vel_pub =
      n.advertise<geometry_msgs::TwistWithCovarianceStamped>("fix_velocity",
                                                             100);

  sensor_msgs::NavSatFix msg;
  sensor_msgs::NavSatStatus status_msg;
  geometry_msgs::TwistWithCovarianceStamped velocity_msg;

  msg.header.frame_id = ros::this_node::getName();
  status_msg.service = 3;  // For GPS+GLONASSS
  int last_fix = 0;

  while (ros::ok()) {
    gps_object.read();
    if (states.fix_count > last_fix) {
      last_fix = states.fix_count;
      ROS_INFO("Status: %u position lat: %f long: %f accuracy: %f\n",
               states.status, states.latitude, states.longitude,
               states.horizontal_accuracy);

      status_msg.status = states.status - 2;
      msg.header.seq = states.fix_count;
      msg.header.stamp = ros::Time::now();
      msg.status = status_msg;
      msg.latitude = states.latitude;
      msg.longitude = states.longitude;
      msg.altitude = states.altitude;
      msg.position_covariance_type = msg.COVARIANCE_TYPE_DIAGONAL_KNOWN;
      msg.position_covariance = {states.horizontal_accuracy, 0, 0, 0,
                                 states.horizontal_accuracy, 0, 0, 0,
                                 states.vertical_accuracy};

      velocity_msg.header = msg.header;
      velocity_msg.twist.twist.linear.x = states.velocity_x;
      velocity_msg.twist.twist.linear.y = states.velocity_y;
      velocity_msg.twist.twist.linear.z = states.velocity_z;
      velocity_msg.twist.twist.angular.z =
          states.ground_course;  // Heading in degrees

      velocity_msg.twist.covariance = {states.horizontal_accuracy,
                                       0,
                                       0,
                                       0,
                                       0,
                                       0,
                                       0,
                                       states.horizontal_accuracy,
                                       0,
                                       0,
                                       0,
                                       0,
                                       0,
                                       0,
                                       states.vertical_accuracy,
                                       0,
                                       0,
                                       0,
                                       0,
                                       0,
                                       0,
                                       1e6,
                                       0,
                                       0,
                                       0,
                                       0,
                                       0,
                                       0,
                                       1e6,
                                       0,
                                       0,
                                       0,
                                       0,
                                       0,
                                       0,
                                       1e6};
      gps_pub.publish(msg);
      gps_vel_pub.publish(velocity_msg);
    }
    ros::spinOnce();
    usleep(1000);  // Wait for 1ms before polling again
  }
  return 0;
}
