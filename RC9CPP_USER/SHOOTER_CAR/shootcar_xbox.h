#ifndef SHOOTCAR_XBOX_H
#define SHOOTCAR_XBOX_H

#ifdef __cplusplus
extern "C"
{
#endif
#include "xbox.h"
#include "chassis.h"
#include "motor.h"
#ifdef __cplusplus
}
#endif
#ifdef __cplusplus


class shootcar_xbox : public xbox, public ITaskProcessor
{
};

#endif
#endif