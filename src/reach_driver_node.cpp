#include "ros/ros.h"
#include "sensor_msgs/NavSatFix.h"
#include "reach_parser.h"

int main(int argc, char **argv)
{	
	GPS_State states;
	GPS_ERB gps_object(states,"/dev/ttyACM0");
	int last_fix = 0;
	
	ros::init(argc, argv, "talker");
	ros::NodeHandle n;
	ros::Publisher gps_pub = n.advertise<sensor_msgs::NavSatFix>("/gps/fix", 100);
	sensor_msgs::NavSatFix msg;
	msg.header.frame_id = "gps";
	sensor_msgs::NavSatStatus status_msg;
	status_msg.service  = 3; // For GPS+GLONASSS
	while(ros::ok())
	{
		gps_object.read();
		if(states.fix_count > last_fix)
		{
			last_fix = states.fix_count;
			printf("Status: %u position lat: %f long: %f accuracy: %f\n", states.status, states.latitude, states.longitude, states.horizontal_accuracy);
			status_msg.status = states.status - 2;
			msg.header.seq = states.fix_count;
			msg.header.stamp = ros::Time::now();			
			msg.status = status_msg;
			msg.latitude = states.latitude;
			msg.longitude = states.longitude;
			msg.altitude = states.altitude;
			msg.position_covariance_type = msg.COVARIANCE_TYPE_DIAGONAL_KNOWN;
			msg.position_covariance = {states.horizontal_accuracy,0,0,
							0,states.horizontal_accuracy,0,
							0,0,states.vertical_accuracy};
			gps_pub.publish(msg);
		}
		usleep(10000); // Wait for 10ms before polling again
	}
	return 0;
}
