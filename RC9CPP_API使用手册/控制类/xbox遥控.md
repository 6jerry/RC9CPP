### `xbox` 基类使用手册

`xbox` 基类是由6Jerry自主研发，为开发自定义 Xbox 手柄遥控逻辑而设计的框架，它提供了底盘控制、按键自定义和摇杆映射计算的功能，能够快速实现复杂的机器人遥控系统。通过继承此基类，用户可以轻松扩展自己的遥控逻辑，并对按键、摇杆操作进行高度自定义。

---

### 1. 功能与代码概述

#### 1.1 主要功能
- **底盘控制**：解析手柄按键和摇杆输入，并控制机器人底盘的运动模式，包括速度调节、模式切换等。
- **按键自定义**：提供一套按键配置系统，允许为每个按键设置不同的行为，如状态切换、递增、递减或自定义操作。
- **摇杆映射计算**：基类内置了 `joymap_compute` 函数，方便用户将摇杆的原始值映射为速度控制或方向控制。
- **支持用户扩展的控制逻辑**：虽然基类并不直接集成 `MFESM`（多标志位编码状态机），但强烈建议用户在开发复杂操控逻辑时使用 `MFESM` 来管理不同的状态。`MFESM` 能够简化复杂标志位的管理，但用户也可以选择自己的方式进行状态控制。

#### 1.2 数据结构概述
为了更好地管理手柄按键、摇杆等数据，基类提供了 `XboxControllerData_t` 数据结构，它用于存储所有按键和摇杆的当前值及其上一次状态：

```cpp
typedef struct
{
    // 按键数据
    bool btnY, btnY_last, btnB, btnB_last, btnA, btnA_last, btnX, btnX_last;
    bool btnShare, btnShare_last, btnStart, btnStart_last, btnSelect, btnSelect_last;
    bool btnXbox, btnXbox_last, btnLB, btnLB_last, btnRB, btnRB_last, btnLS, btnLS_last;
    bool btnRS, btnRS_last, btnDirUp, btnDirup_last, btnDirLeft, btnDirLeft_last;
    bool btnDirRight, btnDirRight_last, btnDirDown, btnDirDown_last;

    // 摇杆及扳机值
    uint16_t joyLHori, joyLVert, joyRHori, joyRVert, trigLT, trigRT;
    float joyLHori_map, joyLVert_map, joyRHori_map, joyRVert_map, trigLT_map, trigRT_map;
} XboxControllerData_t;
```
该结构体管理了所有按键的当前状态及上一次状态，并存储了摇杆与扳机的原始值以及映射值，便于在控制机器人时使用。

---

### 2. 基类的优势

#### 2.1 快速开发与定制
通过 `xbox` 基类，用户可以快速实现遥控逻辑，无需从零开发按键处理逻辑。按键的配置和自定义支持用户根据需求灵活调整，从而大大减少开发时间。

#### 2.2 按键行为的灵活配置
`xbox` 基类提供了多种按键行为，例如状态切换 (`Toggle`)、状态递增 (`Increment`)、状态递减 (`Decrement`) 和自定义操作 (`Custom`)。这种灵活的配置方式使得复杂的控制逻辑变得简单高效。

#### 2.3 摇杆映射计算
基类提供了 `joymap_compute` 函数，能够将手柄的摇杆和扳机的原始值映射为实际的控制量。例如，左右摇杆可以用来控制机器人速度，扳机可以映射为加速控制。这一功能使得基类在各种控制场景下都能灵活使用。

#### 2.4 可扩展的状态管理
虽然 `xbox` 基类不直接集成状态管理逻辑，但基类与 `MFESM` 状态机的集成十分方便，尤其适合处理复杂的状态切换逻辑。`MFESM` 能够高效地管理多标志位状态，简化状态处理的复杂性，用户也可以选择自己喜欢的方式进行状态管理。

---

### 3. 如何使用

#### 3.1 创建自定义类

要使用 `xbox` 基类，首先需要创建一个继承自 `xbox` 的子类，并实现具体的按键逻辑。通过在子类中定义按键配置和状态机初始化，可以快速搭建自定义的遥控逻辑。

##### 示例代码：
```cpp
class xbox_r2n : public xbox, public ITaskProcessor
{
private:
    RC9Protocol *robot_data_chain;

public:
    xbox_r2n(action *ACTION_, chassis *control_chassis_);
    void process_data();  // 实现数据处理
    void chassis_btn_init();  // 按键初始化
    void btnRB_callback() override;  // 自定义按键回调
    void btnXBOX_callback() override; // 自定义按键回调
};
```

#### 3.2 摇杆映射计算

`xbox` 基类提供了 `joymap_compute` 函数，它将摇杆的原始值映射为控制量。此函数会根据摇杆的值进行线性映射，用户可以直接调用此函数来处理摇杆值的变化。

##### 示例代码：
```cpp
void xbox::joymap_compute()
{
    if (xbox_msgs.joyLHori > 31000 && xbox_msgs.joyLHori < 350000)
        xbox_msgs.joyLHori_map = 0.0f;
    else if (xbox_msgs.joyLHori <= 31000)
        xbox_msgs.joyLHori_map = (31000.0f - (float)xbox_msgs.joyLHori) / 31000.0f;
    else
        xbox_msgs.joyLHori_map = (35000.0f - (float)xbox_msgs.joyLHori) / 30535.0f;

    // 同理处理其他摇杆和扳机
}
```
这一功能可以将手柄的摇杆值从原始范围映射为合适的速度或方向控制量，方便在实际控制中应用。

#### 3.3 按键行为与配置

基类通过 `ButtonConfig` 结构体配置按键的行为。按键的行为可以是状态翻转、递增、递减或自定义操作。

##### 按键行为介绍：
- **Toggle**：按下时，状态在 `0` 和 `1` 之间切换，用于控制开关类功能。
- **Increment**：按下时，状态递增，适用于档位或模式选择。
- **Decrement**：按下时，状态递减。
- **Custom**：通过回调函数执行自定义操作。

##### 按键配置示例：
```cpp
void xbox_r2n::chassis_btn_init()
{
    btnAConfig = {
        &xbox_msgs.btnA, &xbox_msgs.btnA_last, &if_point_track_flag, 1,
        ButtonActionType::Toggle, nullptr};

    btnBConfig = {
        &xbox_msgs.btnB, &xbox_msgs.btnB_last, &speed_level, 2,
        ButtonActionType::Increment, nullptr};

    btnXConfig = {
        &xbox_msgs.btnX, &xbox_msgs.btnX_last, &speed_level, 2,
        ButtonActionType::Decrement, nullptr};

    btnRBConfig = {
        &xbox_msgs.btnRB, &xbox_msgs.btnRB_last, &head_locking_flag, 1,
        ButtonActionType::Toggle, &xbox::btnRB_callback};  // 自定义按键回调
}
```

#### 3.4 使用 MFESM 管理复杂状态

虽然基类不强制要求使用 `MFESM`（多标志位编码状态机），但我们强烈建议在开发复杂的状态逻辑时采用 `MFESM`，尤其是在需要处理多个标志位的场景中。`MFESM` 能够将多个标志位的组合映射为一个状态索引，从而极大简化复杂逻辑的处理。MFESM的详细使用教程请去看MFESM的相关文档。

##### 如何集成 `MFESM`：
1. **标志位配置**：首先定义你所需要的标志位，并通过 `FlagConfig` 结构体配置它们的最大值和当前状态指针。
2. **状态机初始化**：调用 `MFESM` 的 `mapStateToIndices` 函数，将特定的标志位组合映射到具体的状态。
3. **状态获取与执行**：使用 `MFESM` 的 `getState` 函数获取当前标志位组合对应的状态，并据此执行相关逻辑。

##### 示例代码：
```cpp
void xbox_r2n::state_machine_init()
{
    // 定义标志位
    FlagConfig flagConfigs[4] = {
        {&world_robot_flag, 1}, {&robot_stop_flag, 1},
        {&if_point_track_flag, 1}, {&if_pure_pus

it, 1}};
    
    EncodingStateMachine stateMachine(flagConfigs, 4);
    
    // 状态映射
    uint16_t worldRemoteIndices[] = {1};
    stateMachine.mapStateToIndices(0, worldRemoteIndices, 1);
    
    uint16_t robotRemoteIndices[] = {0};
    stateMachine.mapStateToIndices(1, robotRemoteIndices, 1);
}
 currentState = stateMachine.getState();
switch (currentState)
    {
    case 0:
        control_chassis->switch_chassis_mode(remote_worldv);
        control_chassis->setworldv(MAX_ROBOT_SPEED_X * xbox_msgs.joyLHori_map, MAX_ROBOT_SPEED_Y * xbox_msgs.joyLVert_map, -MAX_ROBOT_SPEED_W * xbox_msgs.joyRHori_map);
        break;
    case 1:
        control_chassis->switch_chassis_mode(remote_robotv);
        control_chassis->setrobotv(MAX_ROBOT_SPEED_X * xbox_msgs.joyLHori_map, MAX_ROBOT_SPEED_Y * xbox_msgs.joyLVert_map, -MAX_ROBOT_SPEED_W * xbox_msgs.joyRHori_map);
        break;
    case 2:
        control_chassis->switch_chassis_mode(pure_pursuit);
        break;
    case 3:
        control_chassis->switch_chassis_mode(chassis_standby);
        break;
    case 4:
        control_chassis->switch_chassis_mode(chassis_standby);
        break;
    case 255:
        control_chassis->switch_chassis_mode(chassis_standby);
        break;

    default:
        break;
    }
```

---

#### 3.5XBOX数据更新

该Xbox类只负责遥控逻辑的实现，并不包含xbox数据的接收，实际上，xbox数据的接收和解析由一块esp32-s3完成，该mcu与stm32再进行串口通讯，协议为rc9通用协议，也就是说我们想要得到最新的xbox按键遥感数据还需要与一个rc9协议类进行交互，通过基类中的update函数进行交互，在setup程序中，只需在rc9类里添加该xbox类，就可以自动完成数据实时更新

```cpp
RC9Protocol esp32_serial(&huart1, false);//用于与esp32通讯的rc9协议类
xbox_r2n r2_remote(&Action, &r2n_chassis);//xbox遥控类

esp32_serial.addsubscriber(&r2_remote);//建立联系即可实时更新数据
```

by 6Jerry 2024-10-27