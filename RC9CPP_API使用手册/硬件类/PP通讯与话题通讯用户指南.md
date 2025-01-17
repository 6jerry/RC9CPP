# **PP通讯与话题通讯用户指南**

---

## **简介**

在嵌入式系统的开发中，不同模块之间的高效、灵活通信是一个关键问题。为此，6jerry设计了基于 **PP通讯** 和 **话题通讯** 的框架。这套框架通过简单的接口和直观的使用方式，为开发者提供了高效、可靠的模块间数据传输能力。尤其是对于资源有限的嵌入式系统，框架避免了动态内存分配，同时支持多种传输模式，兼顾灵活性和性能。

无论是点对点的通信，还是基于发布-订阅模式的多模块通信，这套框架都能满足需求。

---

### **核心特点**

1. **简单易用：**
   - 用户无需关注底层实现，只需通过简单的接口完成模块间数据传输。
   - 对于话题通讯，开发者仅需指定话题名称，系统自动完成订阅与发布关系的维护。

2. **强大的通用性：**
   - 支持任意数据类型的传输（通过 `void*` 指针），包括整型、浮点型、数组和复杂的结构体数据。
   - 可满足各种模块间数据交换需求。

3. **多种传输模式：**
   - **同步发送（SYN）：** 实时、直接调用目标模块处理函数，适合低延时需求场景。
   - **异步发送（ASYN）：** 使用消息队列进行非阻塞发送，避免因阻塞影响系统性能。
   - **异步覆盖发送（ASYNOVERWRITE）：** 保持队列中只存储最新消息，适合只需最新状态的场景。

4. **高效且嵌入式友好：**
   - 采用静态资源分配，无动态内存操作，避免嵌入式系统中常见的内存泄漏问题。
   - 初始化时完成话题名称到 `topicID` 的映射，发布和接收时直接通过 ID 查找，大幅提高运行效率。

5. **灵活扩展：**
   - 一个模块可以同时发布多个话题或订阅多个话题。
   - 易于适配复杂的模块化应用场景。

---

## **框架结构与概念**

### **PP通讯**

#### **什么是PP通讯？**

PP（Point-to-Point）通讯是一个基础的模块间通讯机制，允许开发者通过 IP 和 MAC 地址，向任意目标模块发送数据。PP通讯支持 3 种传输模式：

- **同步发送（SYN）：** 数据直接传输到目标模块，立即调用其处理函数。
- **异步发送（ASYN）：** 数据通过消息队列传输，非阻塞，适合长时间运行任务。
- **异步覆盖发送（ASYNOVERWRITE）：** 仅保存最新数据，旧数据会被覆盖。

#### **PP通讯的优点**

- **灵活性高：** 用户可自由选择传输模式，满足不同的实时性与可靠性需求。
- **传输任意数据：** `void*` 指针使得数据传输类型完全通用。
- **简单实现：** 用户只需实现 `msgin` 和 `msgout` 函数，管理自身模块的数据接收与发送逻辑。

---

### **话题通讯**

#### **什么是话题通讯？**

话题通讯是基于 PP 通讯实现的发布-订阅机制，适合多模块间的广播式数据传输。开发者只需指定话题名称即可完成发布与订阅关系的建立。其底层通过 `topicID` 高效查找订阅者，并通过 PP 通讯完成数据传输。

#### **话题通讯的优点**

1. **简单直观：**
   - 用户无需手动维护模块间的 IP 和 MAC 地址映射。
   - 通过话题名称（如 `"temperature"`）即可完成复杂的多模块通讯。

2. **性能高效：**
   - 初始化时处理字符串话题名称，运行时直接通过 `topicID` 查找映射关系，避免高频字符串操作带来的性能开销。

3. **灵活强大：**
   - 支持一个模块同时发布多个话题。
   - 支持一个模块订阅多个话题，轻松构建复杂的数据交互网络。

---

## **使用方法**

### **PP通讯的使用**

#### **步骤：**

1. **继承 `rcnode` 并实现接口：**
   - 必须实现 `msgin` 函数，用于接收消息。
   - `msgout` 函数（可选）用于数据读取。

2. **初始化模块：**
   - 使用 `rcninit` 方法注册模块的 MAC 地址。

3. **发送和接收数据：**
   - 使用 `ppsend_Syn`、`ppsend_Asyn`、`ppsend_AsynOverwrite` 完成发送。
   - 使用 `ppget_Asyn` 或 `ppget_AsynOverwrite` 接收消息。

#### **示例代码：**

```cpp
class Sensor : public rcnode {
public:
    uint8_t msgin(uint8_t rcnID_, const void *data) override {//接收回调函数,用户根据需求自己实现
        if (rcnID_ == 1) {
            float value = *reinterpret_cast<const float*>(data);
            printf("Sensor received: %.2f\n", value);
        }
        return 1;
    }
};

class Actuator : public rcnode {
public:
    void sendCommand(float command) {
        ppsend_Syn(1, 2, 1, &command);//同步发送函数，使用起来较为复杂，因为需要指定mac和ip，推荐使用更上层的话题通讯，其对于pp通讯进行了封装，能通过直观易懂的话题名称建立起地址对应关系，无需用户关注地址
    }
};
```

---

### **话题通讯的使用**

#### **步骤：**

1. **初始化发布者和订阅者：**
   - 使用 `publisher::init` 方法注册话题。
   - 使用 `subscriber::init` 方法订阅话题。

2. **发布数据：**
   - 调用 `publish` 方法，将数据广播到所有订阅者。

3. **接收数据：**
   - 订阅者调用 `hearfromtopic` 方法处理接收的数据。

#### **示例代码：**

```cpp
class TemperatureSensor : public rcnode {
private:
    publisher tempPub;

public:
    void initPublisher() {
        tempPub.init("temperature", SYN, this);//初始化话题发布者
    }

    void sendTemperature(float temp) {
        tempPub.publish(1, &temp);//发布任意数据到该话题
    }
};

class Display : public rcnode {
private:
    subscriber tempSub;

public:
    void initSubscriber() {
        tempSub.init("temperature", ASYN, this);//初始化话题接收者
    }

    uint8_t msgin(uint8_t rcnID_, const void *data) override {//接收回调函数
        if (rcnID_ == 1) {
            float value = *reinterpret_cast<const float*>(data);
            printf("Display: Temperature %.2f\n", value);
            return 1;
        }
        return 0;
    }

    void checkTemperature() {
        tempSub.hearfromtopic();
    }
};
```

---

## **底层实现原理**

1. **PP通讯：**
   - 使用 `rcnode` 类维护模块的 MAC 地址映射关系。
   - 基于 FreeRTOS 消息队列实现异步和异步覆盖发送。

2. **话题通讯：**
   - `rcncore` 管理所有话题的注册和映射。
   - 初始化时通过话题名称生成 `topicID`，运行时通过 `topicID` 查找订阅者。

---

## **优点总结**

1. **用户友好：**
   - 用户无需管理 IP 和 MAC 地址。
   - 话题名称直观，接口简单，学习成本低。

2. **强大的灵活性：**
   - 支持多种发送模式（同步、异步、覆盖）。
   - 模块可同时发布和订阅多个话题。

3. **嵌入式环境优化：**
   - 使用静态数组，无动态内存分配。
   - 初始化处理字符串，运行时性能高效。

---

## **适用场景**

- **实时性要求高的模块通讯：** 机器人控制、传感器采集与处理。
- **模块化系统：** 复杂嵌入式应用中，多个模块间的解耦数据传输。





## **未来的发展计划**

### **1. 跨主控通讯**

目前，框架实现的 PP 通讯和话题通讯基于单个主控内部的模块间通信。未来，我们计划扩展到跨 STM32 主控的模块通信，支持多个主控间的数据交互。
具体改进方向：

- 基于 IP 协议建立模块间通信，允许不同主控设备共享话题和服务。
- 提供模块发现功能，自动识别并连接网络中的其他主控。

### **2. 服务通讯**

除了发布-订阅模式，我们计划新增 **服务通讯** 机制，用于请求-响应式的数据交互。

- 发布者（服务端）可提供服务接口。
- 订阅者（客户端）可以通过特定接口调用服务，获取数据或触发动作。

**应用场景：**

- 主控设备 A 请求主控设备 B 的传感器数据。
- 机器人模块之间的任务分配与执行。