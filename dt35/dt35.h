
/*
 * @author loopgad
 * @contact 3280646246@qq.com
 * @license MIT License
 *
 * Copyright (c) 2024 loopgad
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

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