#ifndef AIR_JOY_TEST_H
#define AIR_JOY_TEST_H


#ifdef __cplusplus
extern "C"
{
#endif

#include "air_joy.h"
#include "TaskManager.h"
#include "robot_chassis.h"
	
#ifdef __cplusplus
}
#endif

#ifdef __cplusplus
class AirJoyTest : public ITaskProcessor, public AirJoy, public chassis_user
{
    public:
    void process_data();
};
#endif

#endif /* AIR_JOY_TEST_H */