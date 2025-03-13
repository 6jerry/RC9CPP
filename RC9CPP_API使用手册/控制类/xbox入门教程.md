# `xbox` 基类详解与使用文档

大家好！欢迎深入探索我的 `xbox` 基类！这是一个为Xbox控制器设计的强大工具，专为机器人控制、调试和算法测试打造。它能帮你轻松接收Xbox控制器的输入（按键、摇杆、扳机），并将其转化为机器人系统的控制信号。最重要的是，它设计得超级灵活，通过继承和配置，你可以打造出完全符合自己需求的自定义控制器。

这份文档的目标是让开发者（也就是你！）彻底掌握 `xbox` 基类的功能，尤其是按键配置系统。我会用轻松的语气，结合详细的说明和丰富的代码示例，确保你看完后不仅能用起来，还能随心所欲地配置按键行为，真正实现自定义控制器的能力。

------

## 文档结构

- **1. 基类总览**：了解 `xbox` 基类的功能和设计理念。
- **2. API详解**：逐一讲解核心函数和数据结构。
- **3. 按键配置系统深度解析**：重点讲解如何配置按键的各种行为。
- **4. 扩展与自定义指南**：教你如何基于基类编写自己的控制器。
- **5. 完整使用示例**：从头到尾展示一个完整案例。

准备好了吗？咱们开始吧！

------

## 1. 基类总览

### 1.1 功能简介

`xbox` 基类是我的机器人控制架构中的核心模块，负责以下任务：

- **接收Xbox控制器数据**：通过 `RC9Protocol` 协议，实时获取按键状态、摇杆位置和扳机值。
- **按键行为处理**：提供灵活的按键配置系统，支持翻转（Toggle）、递增（Increment）、递减（Decrement）和自定义（Custom）四种行为。
- **输入映射**：将原始的摇杆和扳机值标准化为-1到1或0到1，方便生成控制信号。
- **基础底盘控制**：内置默认速度参数，适合简单的机器人移动。

它继承自 `RC9subscriber`，通过订阅 `RC9Protocol` 的数据流更新状态。基类的设计强调**通用性**和**可扩展性**，只实现了基础功能，其他特性（比如机械臂控制、调试模式）需要你通过继承来实现。

### 1.2 适用场景

- **遥控机器人**：用Xbox操控机器人移动。
- **调试工具**：切换模式、调整参数。
- **算法测试**：模拟输入信号，验证算法。

### 1.3 设计亮点

- **简单易用**：API直观，配置清晰。
- **高度灵活**：支持自定义按键行为和功能扩展。
- **实时高效**：与 `RC9Protocol` 配合，数据处理快。

------

## 2. API详解

### 2.1 数据结构

#### 2.1.1 `XboxControllerData_t`

```cpp
typedef struct {
    // 按键数据（示例，完整结构体包含所有按键）
    bool btnY;         // Y键当前状态
    bool btnY_last;    // Y键上一次状态
    bool btnB;         // B键当前状态
    bool btnB_last;    // B键上一次状态
    // ... 其他按键（A, X, Share, Start, Select, Xbox, LB, RB, LS, RS, 方向键）
    // 原始输入值
    uint16_t joyLHori;  // 左摇杆水平值 (0-65535)
    uint16_t joyLVert;  // 左摇杆垂直值 (0-65535)
    uint16_t joyRHori;  // 右摇杆水平值 (0-65535)
    uint16_t joyRVert;  // 右摇杆垂直值 (0-65535)
    uint16_t trigLT;    // 左扳机值 (0-1023)
    uint16_t trigRT;    // 右扳机值 (0-1023)
    // 映射后的值
    float joyLHori_map;  // 左摇杆水平映射值 (-1到1)
    float joyLVert_map;  // 左摇杆垂直映射值 (-1到1)
    float joyRHori_map;  // 右摇杆水平映射值 (-1到1)
    float joyRVert_map;  // 右摇杆垂直映射值 (-1到1)
    float trigLT_map;    // 左扳机映射值 (0到1)
    float trigRT_map;    // 右扳机映射值 (0到1)
} XboxControllerData_t;
```

- **作用**：存储所有控制器输入数据。原始值由硬件提供，映射值由 `joymap_compute` 生成。
- **访问方式**：通过 `xbox` 类的公共成员 `xbox_msgs` 访问，例如 `my_xbox.xbox_msgs.btnA`。

------

### 2.2 构造函数

```cpp
xbox();
```

- 功能

  ：初始化 

  ```
  xbox
  ```

   实例，设置默认速度参数：

  - `MAX_ROBOT_SPEED_Y = 1.50f`（前后速度）。
  - `MAX_ROBOT_SPEED_X = 1.50f`（左右速度）。
  - `MAX_ROBOT_SPEED_W = 3.60f`（角速度）。

- **用法**：直接创建对象即可。

- 代码示例

  ：

  ```cpp
  xbox my_xbox;  // 创建基础实例
  ```

------

### 2.3 核心函数

#### 2.3.1 `DataReceivedCallback`

```cpp
void DataReceivedCallback(const uint8_t *byteData, const float *floatData, uint8_t id, uint16_t byteCount) override;
```

- **功能**：从 `RC9Protocol` 接收数据，更新 `xbox_msgs`。

- 参数

  ：

  - `byteData`：字节数据（按键和原始值）。
  - `floatData`：浮点数据（未使用）。
  - `id`：数据帧ID。
  - `byteCount`：字节数（通常为28）。

- **用法**：由 `RC9Protocol` 自动调用，可重写以自定义解析。

- 代码示例

  ：

  ```cpp
  class MyXbox : public xbox {
  public:
      void DataReceivedCallback(const uint8_t *byteData, const float *floatData, uint8_t id, uint16_t byteCount) override {
          xbox::DataReceivedCallback(byteData, floatData, id, byteCount);  // 调用基类解析
          printf("Data received!\n");  // 添加自定义逻辑
      }
  };
  ```

#### 2.3.2 `joymap_compute`

```cpp
void joymap_compute();
```

- 功能

  ：将原始输入映射为标准化值：

  - 摇杆：31000-35000为死区，小于31000映射为-1到0，大于35000映射为0到1。
  - 扳机：0-1023线性映射为0到1。

- **返回值**：无，更新 `xbox_msgs` 的 `_map` 字段。

- **用法**：在需要标准化输入时调用。

- 代码示例

  ：

  ```cpp
  xbox my_xbox;
  my_xbox.joymap_compute();
  float speed = my_xbox.xbox_msgs.joyLVert_map * my_xbox.MAX_ROBOT_SPEED_Y;
  ```

#### 2.3.3 `handleButton`

```cpp
void handleButton(ButtonConfig &config);
```

- **功能**：处理按键事件，根据配置执行动作。

- 参数

  ：

  - `config`：按键配置结构体（详见下节）。

- **用法**：为每个按键配置 `ButtonConfig`，然后调用此函数。

- 代码示例

  ：

  ```cpp
  xbox my_xbox;
  uint8_t flag = 0;
  my_xbox.btnAConfig = {
      &my_xbox.xbox_msgs.btnA,
      &my_xbox.xbox_msgs.btnA_last,
      &flag,
      1,
      xbox::ButtonActionType::Toggle,
      nullptr
  };
  my_xbox.handleButton(my_xbox.btnAConfig);
  ```

------

## 3. 按键配置系统深度解析

按键配置是 `xbox` 基类的核心亮点，让你可以为每个按键定义独特的行为。下面我会详细讲解 `ButtonConfig` 结构体的每个成员，以及如何配置四种行为（`Toggle`、`Increment`、`Decrement`、`Custom`），确保你能灵活运用。

### 3.1 `ButtonConfig` 结构体详解

```cpp
struct ButtonConfig {
    bool *currentState;           // 当前按键状态的指针
    bool *lastState;              // 上次按键状态的指针
    uint8_t *toggleState;         // 要操作的状态变量的指针
    uint8_t maxState;             // 状态变量的最大值
    ButtonActionType actionType;  // 按键行为类型
    void (xbox::*customAction)(); // 自定义动作的函数指针
};
```

- **预定义实例**：`xbox` 类为每个按键提供了配置实例，如 `btnAConfig`、`btnBConfig` 等，直接使用即可。

- 成员逐一说明

  ：

  - **`currentState`**：指向 `xbox_msgs` 中的按键状态（如 `&xbox_msgs.btnA`），由 `DataReceivedCallback` 更新。

  - **`lastState`**：指向上一状态（如 `&xbox_msgs.btnA_last`），用于检测按键上升沿（从0到1）。

  - **`toggleState`**：指向你要操作的变量（`uint8_t` 类型），比如一个模式标志。`Custom` 行为可以设为 `nullptr`。

  - **`maxState`**：`toggleState` 的最大值，限制变量范围。

  - `actionType`

    ：枚举类型，定义按键行为：

    - `Toggle`：翻转状态。
    - `Increment`：递增状态。
    - `Decrement`：递减状态。
    - `Custom`：执行自定义函数。

  - **`customAction`**：指向 `xbox` 类成员函数（无参数、无返回值），仅在 `Custom` 时使用。

------

### 3.2 配置按键行为的步骤

要为某个按键配置行为，只需以下几步：

1. **选择按键**：决定用哪个按键（比如 `btnAConfig`）。
2. **定义变量**（如需要）：声明一个 `uint8_t` 变量存储状态。
3. **设置 `ButtonConfig`**：填充结构体字段。
4. **调用 `handleButton`**：在循环中处理按键事件。

下面我会详细讲解每种行为的配置方法，带你一步步学会。

------

### 3.3 四种行为配置详解

#### 3.3.1 `Toggle`（翻转）

- **作用**：每次按下按键，`toggleState` 在0到 `maxState` 之间循环切换。

- **适用场景**：开关功能，比如启动/停止模式。

- 配置方法

  ：

  - `currentState` 和 `lastState` 指向对应按键状态。
  - `toggleState` 指向你的状态变量。
  - `maxState` 设置状态上限。
  - `actionType` 设为 `Toggle`。
  - `customAction` 设为 `nullptr`。

- 代码示例

  ：

  ```cpp
  xbox my_xbox;
  uint8_t mode = 0;  // 0:关闭, 1:开启
  my_xbox.btnAConfig = {
      &my_xbox.xbox_msgs.btnA,      // 当前状态
      &my_xbox.xbox_msgs.btnA_last, // 上次状态
      &mode,                        // 操作的变量
      1,                            // 最大值为1（0和1两种状态）
      xbox::ButtonActionType::Toggle,
      nullptr
  };
  my_xbox.handleButton(my_xbox.btnAConfig);  // 按A键切换mode
  printf("Mode: %d\n", mode);  // 输出当前模式
  ```

- **效果**：按一次A键，`mode` 从0变为1，再按变为0。

#### 3.3.2 `Increment`（递增）

- **作用**：每次按下按键，`toggleState` 加1，直到 `maxState`。

- **适用场景**：调节级别，比如速度等级（低、中、高）。

- 配置方法

  ：

  - 同上，但 `actionType` 设为 `Increment`。

- 代码示例

  ：

  ```cpp
  xbox my_xbox;
  uint8_t speed_level = 0;  // 0:低速, 1:中速, 2:高速
  my_xbox.btnBConfig = {
      &my_xbox.xbox_msgs.btnB,
      &my_xbox.xbox_msgs.btnB_last,
      &speed_level,
      2,  // 最大值为2
      xbox::ButtonActionType::Increment,
      nullptr
  };
  my_xbox.handleButton(my_xbox.btnBConfig);  // 按B键增加speed_level
  printf("Speed level: %d\n", speed_level);
  ```

- **效果**：按B键，`speed_level` 从0增到1，再到2，达到2后不再增加。

#### 3.3.3 `Decrement`（递减）

- **作用**：每次按下按键，`toggleState` 减1，直到0。

- **适用场景**：与 `Increment` 配合，实现双向调节。

- 配置方法

  ：

  - 同上，但 `actionType` 设为 `Decrement`。

- 代码示例

  ：

  ```cpp
  xbox my_xbox;
  uint8_t speed_level = 2;  // 初始值为高速
  my_xbox.btnXConfig = {
      &my_xbox.xbox_msgs.btnX,
      &my_xbox.xbox_msgs.btnX_last,
      &speed_level,
      2,  // 最大值为2
      xbox::ButtonActionType::Decrement,
      nullptr
  };
  my_xbox.handleButton(my_xbox.btnXConfig);  // 按X键减少speed_level
  printf("Speed level: %d\n", speed_level);
  ```

- **效果**：按X键，`speed_level` 从2减到1，再到0，达到0后不再减少。

#### 3.3.4 `Custom`（自定义）

- **作用**：执行你定义的函数，实现任意操作。

- **适用场景**：需要复杂逻辑，比如重启系统、记录位置。

- 配置方法

  ：

  - `toggleState` 和 `maxState` 可设为 `nullptr` 和 0。
  - `actionType` 设为 `Custom`。
  - `customAction` 指向自定义成员函数。

- 代码示例

  ：

  ```cpp
  class MyXbox : public xbox {
  public:
      void resetSystem() {
          printf("System reset!\n");  // 自定义操作
      }
  
      void setup() {
          btnStartConfig = {
              &xbox_msgs.btnStart,
              &xbox_msgs.btnStart_last,
              nullptr,             // 不需要状态变量
              0,
              ButtonActionType::Custom,
              &MyXbox::resetSystem // 指向成员函数
          };
      }
  
      void run() {
          handleButton(btnStartConfig);  // 按Start键执行resetSystem
      }
  };
  
  int main() {
      MyXbox my_xbox;
      my_xbox.setup();
      my_xbox.run();
      return 0;
  }
  ```

- **效果**：按Start键，打印“System reset!”。

------

### 3.4 按键配置注意事项

- **状态更新**：`handleButton` 依赖 `lastState` 检测变化，确保在循环中定期调用。
- **自定义函数限制**：`customAction` 必须是 `xbox` 类的成员函数，无参数、无返回值。
- **变量类型**：`toggleState` 是 `uint8_t*`，若需其他类型，可用 `Custom` 行为在函数中处理。
- **多按键配置**：可以为多个按键绑定行为，但注意变量共享可能导致冲突。

------

### 3.5 组合使用示例

你可以为不同按键配置不同行为，实现复杂控制逻辑。

**代码示例**：

```cpp
class MyXbox : public xbox {
public:
    uint8_t power = 0;      // 开关
    uint8_t level = 1;      // 等级（0-3）
    void reset() {
        power = 0;
        level = 1;
        printf("Reset!\n");
    }

    void setup() {
        btnAConfig = {&xbox_msgs.btnA, &xbox_msgs.btnA_last, &power, 1, ButtonActionType::Toggle, nullptr};
        btnBConfig = {&xbox_msgs.btnB, &xbox_msgs.btnB_last, &level, 3, ButtonActionType::Increment, nullptr};
        btnXConfig = {&xbox_msgs.btnX, &xbox_msgs.btnX_last, &level, 3, ButtonActionType::Decrement, nullptr};
        btnStartConfig = {&xbox_msgs.btnStart, &xbox_msgs.btnStart_last, nullptr, 0, ButtonActionType::Custom, &MyXbox::reset};
    }

    void run() {
        handleButton(btnAConfig);   // A键开关
        handleButton(btnBConfig);   // B键增加等级
        handleButton(btnXConfig);   // X键减少等级
        handleButton(btnStartConfig); // Start键重置
        printf("Power: %d, Level: %d\n", power, level);
    }
};

int main() {
    MyXbox my_xbox;
    my_xbox.setup();
    my_xbox.run();
    return 0;
}
```

- 效果

  ：

  - 按A键切换 `power`（0或1）。
  - 按B键增加 `level`，按X键减少。
  - 按Start键重置所有状态。

------

## 4. 扩展与自定义指南

### 4.1 添加新功能

通过继承 `xbox`，实现特定逻辑。

**代码示例**：

```cpp
class MyXbox : public xbox {
private:
    float robot_speed = 0.0f;

public:
    void process_data() {
        joymap_compute();
        robot_speed = xbox_msgs.joyLVert_map * MAX_ROBOT_SPEED_Y;
        printf("Speed: %.2f\n", robot_speed);
    }
};
```

### 4.2 集成硬件

绑定硬件对象并控制。

**代码示例**：

```cpp
class MyXbox : public xbox {
private:
    power_motor* motor;

public:
    MyXbox(power_motor* m) : motor(m) {}
    void process_data() {
        joymap_compute();
        motor->set_rpm(xbox_msgs.trigRT_map * 1000.0f);
    }
};
```

------

## 5. 完整使用示例

```cpp
#include "xbox.h"

class MyXbox : public xbox {
public:
    uint8_t mode = 0;  // 模式开关

    void setup() {
        btnAConfig = {
            &xbox_msgs.btnA,
            &xbox_msgs.btnA_last,
            &mode,
            1,
            ButtonActionType::Toggle,
            nullptr
        };
    }

    void run() {
        RC9Protocol protocol(uart, &huart1);  // 假设使用UART1
        addport(&protocol);  // 绑定通信
        while (1) {
            joymap_compute();
            handleButton(btnAConfig);
            float speed = xbox_msgs.joyLVert_map * MAX_ROBOT_SPEED_Y;
            printf("Mode: %d, Speed: %.2f\n", mode, speed);
        }
    }
};

int main() {
    MyXbox my_xbox;
    my_xbox.setup();
    my_xbox.run();
    return 0;
}
```

------

## 6. 总结

`xbox` 基类是一个灵活的工具，通过它的按键配置系统，你可以为每个按键定义独特的行为。希望这份详细文档能让你彻底掌握它的用法，并轻松编写自己的自定义控制器。有问题随时找我聊哦！快去试试吧！