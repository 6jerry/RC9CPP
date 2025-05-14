#include "air_joy_test.h"

void AirJoyTest:: process_data()
{
    joymap_compute();
		if(Joy_msgs.SWA_map == 2)
		{
			Vector2D tvel_((1.0f * Joy_msgs.LEFT_X_map), (1.0f * Joy_msgs.LEFT_Y_map));

			set_RobotVel(tvel_, 0);
			set_RobotW(-(1.0f * Joy_msgs.RIGHT_X_map), 0);
			rst_state();
		}
	
}