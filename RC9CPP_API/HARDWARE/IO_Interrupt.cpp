#include "IO_Interrupt.h""

GPIODevice* GPIODevice::instances_[MAX_GPIO_INSTANCES] = {nullptr};
int GPIODevice::instanceCount_ = 0;

GPIODevice::GPIODevice() 
{
    registerInstance(this);
    initGPIOInterrupt();
}

void GPIODevice::registerInstance(GPIODevice *instance)
{
    if (instanceCount_ < MAX_GPIO_INSTANCES) {
        instances_[instanceCount_++] = instance;
    }
}

void GPIODevice::initGPIOInterrupt()
{

}


extern "C" void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_Pin);
    for (int i = 0; i < GPIODevice::instanceCount_; ++i) {
        GPIODevice* instance = GPIODevice::instances_[i];
        if (instance->pin == GPIO_Pin) {
            instance->handleInterrupt();
        }
    }
}
