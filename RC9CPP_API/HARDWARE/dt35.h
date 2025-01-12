#pragma once

/********************************INCLUDE DIRETORIES*********************************/
#include "RC9Protocol.h"
#include <cstdint>
/********************************INCLUDE DIRETORIES END*********************************/


/*****************************USER DEFINE*******************************/
#define total_channel 8
/****************************USER DEFINE END********************************/

union recieveData
{
	int d;
	unsigned char data[4];
};

/*****************************USING SINGLE INSTANCE******************************/
class dt35 : public RC9Protocol{
private:
    uint32_t dt35_data[6];
    recieveData dr_[6];
public:
    dt35(UART_HandleTypeDef *huart, bool enableCrcCheck);
    void process_data();
};
/*****************************USING SINGLE INSTANCE END******************************/