// #include "ros/ros.h"
#include "reach_parser.h"

int main(int argc, char **argv)
{	
	GPS_State states;
	GPS_ERB gps_object(states,"/dev/ttyACM0");
	int last_fix = 0;
	while(1)
	{
		gps_object.read();
		if(states.fix_count > last_fix)
		{
			last_fix = states.fix_count;
			printf("Status: %d Sats: %d position lat: %f long: %f accuracy: %f\n", states.status, states.num_sats, states.latitude, states.longitude, states.horizontal_accuracy);
		}
		usleep(10000); // Wait for 10ms before polling again
	}
	return 0;
}