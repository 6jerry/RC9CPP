#include "R3_testxbox.h"

void R3_xbox::mode_1()
{
   //shoot_motor_1->send_rpm(rpm);
  // shoot_motor_2->send_rpm(rpm);
	  shoot_motor_1->set_current(c);
	  shoot_motor_2->set_current(c);
	//  shoot_motor_1->send_rpm(move_rpm * xbox_msgs.joyLVert_map);
 //   shoot_motor_2->send_rpm(move_rpm * xbox_msgs.joyLVert_map);
	  if(read_io() == true)
		{
		    shoot_motor_1->send_rpm(0.0f);
				shoot_motor_2->send_rpm(0.0f);
			  mode_flag = 2;
		}
	 
}

void R3_xbox::mode_2()
{
	
	      shoot_motor_1->send_rpm(0.0f);
				shoot_motor_2->send_rpm(0.0f);
    //shoot_motor_1->send_rpm(move_rpm * xbox_msgs.joyLVert_map);
    //shoot_motor_2->send_rpm(move_rpm * xbox_msgs.joyLVert_map);
	
	 
}

void R3_xbox::not_start()
{
    shoot_motor_1->send_rpm(0.0f);
    shoot_motor_2->send_rpm(0.0f);
}


bool R3_xbox::read_io()
{
     return gate.is_finish();

}

//void R3_xbox::add_io(GPIO_TypeDef  *stop_port_, uint16_t  stop_pin_)
//{
//stop_port = stop_port_;
//stop_pin_ = stop_pin_;
//}

photogate_shoot::photogate_shoot()
{

}

void photogate_shoot::handleInterrupt() 
{
  flag = 1;
}
void photogate_shoot::add_io_interrupt(GPIO_TypeDef *port, uint16_t pin) 
{
    port_ = port;
	  pin_ = pin;

}
 bool photogate_shoot::is_finish()
{
   if(flag)
		{
		  flag = 0;
			return true;
		}else{
		  return false;
		}
   
}