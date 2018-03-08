// #include "ros/ros.h"
#include "reach_parser.h"

int main(int argc, char **argv)
{	
	GPS_State states;
	AP_GPS_ERB gps_object(states,"/dev/ttyACM0");
	while(1)
	{
		gps_object.read();
		if(states.status != NO_FIX && states.status != NO_GPS)
			printf("Status: %d position lat: %f long: %f accuracy: %f\n", states.status, states.latitude, states.longitude, states.horizontal_accuracy);
		usleep(100000);
	}
	return 0;
}