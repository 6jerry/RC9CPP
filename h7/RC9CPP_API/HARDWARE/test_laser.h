
/*
 * @author loopgad
 * @contact 3280646246@qq.com
 * @license MIT License
 *
 * Copyright (c) 2025 loopgad
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

// laser_processor.h
#ifndef TEST_LASER_H
#define TEST_LASER_H


#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdint.h>
#include "Serial_device.h"
#include "imu.h"
#include "math.h"
#include "filter.h"
#ifdef __cplusplus
}
#endif

#ifdef __cplusplus

/**************must*************/
// BAUD_RATE: 19200
// GPIO setting: GPIO_PULLUP
/******************************/

class Laser : public SerialDevice, public imu
{
public:
    enum parse_status{
        head0,
        head1,
        head2,
        head3,
        head4,
        head5,
        data,
        qua,
        crc
    };
	KalmanFilter filter;
    volatile parse_status status = head0;
    float distance = 0.0f;
    Laser(UART_HandleTypeDef *huart);
	void init(void);
    float get_distance(void) override;
    void imu_rst() override;
    void handleReceiveData(uint8_t byte);
    const uint8_t cmd_init0[10] = {0xAA, 0x00, 0x00, 0x20, 0x00, 0x01, 0x00, 0x00, 0x21};
	const uint8_t cmd_init1[10] = {0xAA, 0x00, 0x00, 0x20, 0x00, 0x01, 0x00, 0x04, 0x25};
};

#endif
#endif