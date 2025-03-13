# PID控制器类详细使用说明文档

大家好！欢迎深入了解我的电机控制架构中的PID控制器家族！这份文档是为我的三种PID类——`pid`、`superpid`和`IncrePID`——量身打造的详细使用指南。这些类已经在我的电机控制系统中得到了广泛应用（比如M3508电机），帮助我实现了精准的速度和位置控制。接下来，我会以轻松的风格，带你们逐一探索每个类的API函数，包括它们的参数、返回值、使用场景和代码示例，确保你们能快速上手并灵活运用。

文档会按以下结构展开：

- **类简介**：简单介绍每个类的特点和适用场景。
- **API函数详解**：详细说明每个公共函数的作用、参数、返回值和用法。
- **完整使用示例**：提供实际代码，展示如何在电机控制中使用这些类。

准备好了吗？咱们开始吧！

------

## 1. `pid` 类 - 基础版PID控制器

### 1.1 类简介

`pid` 是一个经典的位置式PID控制器，集成了积分限幅、输出限幅、死区处理、积分分离和微分滤波等实用功能。它简单易用，适合大多数普通的控制任务，比如电机转速或位置的初级调节。如果你刚开始接触PID，或者需求不复杂，这个类是个很好的起点。

------

### 1.2 API函数详解

#### 1.2.1 构造函数

```cpp
pid(float kp = 0.0f, float ki = 0.0f, float kd = 0.0f, float integral_limit = 0.0f, float output_limit = 0.0f, float deadzone = 0.0f, float integral_separation_threshold = 0.0f);
```

- **功能**：创建并初始化一个`pid`实例，设置基本的控制参数。

- 参数

  ：

  - `kp`：比例增益（默认0.0f），决定对当前误差的响应强度。
  - `ki`：积分增益（默认0.0f），处理累积误差。
  - `kd`：微分增益（默认0.0f），预测误差变化趋势。
  - `integral_limit`：积分限幅值（默认0.0f，0表示无限制），防止积分过大。
  - `output_limit`：输出限幅值（默认0.0f，0表示无限制），保护硬件。
  - `deadzone`：死区阈值（默认0.0f），误差在此范围内不响应。
  - `integral_separation_threshold`：积分分离阈值（默认0.0f），误差过大时暂停积分。

- **返回值**：无（构造函数无返回值）。

- 代码示例

  ：

  ```cpp
  pid my_pid(1.0f, 0.1f, 0.05f, 100.0f, 500.0f, 0.01f, 10.0f);
  ```

  - 创建一个PID实例，比例增益为1.0，积分限幅为100，输出限幅为500。

#### 1.2.2 `PID_SetParameters`

```cpp
void PID_SetParameters(float kp_, float ki_, float kd_);
```

- **功能**：动态调整PID的增益参数（kp、ki、kd），无需重新配置其他参数。

- 参数

  ：

  - `kp_`：新的比例增益。
  - `ki_`：新的积分增益。
  - `kd_`：新的微分增益。

- **返回值**：无。

- 代码示例

  ：

  ```cpp
  my_pid.PID_SetParameters(2.0f, 0.2f, 0.1f);
  ```

  - 将比例增益改为2.0，积分增益改为0.2，微分增益改为0.1。

#### 1.2.3 `ConfigAll`

```cpp
void ConfigAll(float kp_, float ki_, float kd_, float integral_limit_, float output_limit_, float deadzone_, float integral_separation_threshold_, float filter_a = 1.0f);
```

- **功能**：一次性配置所有参数，包括增益和控制特性，相当于“重置”PID。

- 参数

  ：

  - `kp_`：比例增益。
  - `ki_`：积分增益。
  - `kd_`：微分增益。
  - `integral_limit_`：积分限幅值。
  - `output_limit_`：输出限幅值。
  - `deadzone_`：死区阈值。
  - `integral_separation_threshold_`：积分分离阈值。
  - `filter_a`：微分项低通滤波器的alpha值（默认1.0f，无滤波），范围0到1，越小滤波越强。

- **返回值**：无。

- 代码示例

  ：

  ```cpp
  my_pid.ConfigAll(2.0f, 0.2f, 0.1f, 200.0f, 600.0f, 0.02f, 15.0f, 0.8f);
  ```

  - 调整所有参数，并将微分滤波强度设为0.8。

#### 1.2.4 `PID_Compute`

```cpp
float PID_Compute(float input);
```

- **功能**：根据当前测量值计算PID输出，通常用于实时控制。

- 参数

  ：

  - `input`：当前系统的测量值（比如电机转速或位置）。

- **返回值**：PID计算后的输出值（比如电流或PWM值）。

- 代码示例

  ：

  ```cpp
  float current_rpm = 90.0f;
  float output = my_pid.PID_Compute(current_rpm);
  ```

  - 输入当前转速90 RPM，计算控制输出。

#### 1.2.5 `PID_ComputeError`

```cpp
float PID_ComputeError(float error_);
```

- **功能**：直接传入误差值计算PID输出，适合已知误差的情况。

- 参数

  ：

  - `error_`：预先计算的误差（目标值减测量值）。

- **返回值**：PID计算后的输出值。

- 代码示例

  ：

  ```cpp
  float error = 10.0f;
  float output = my_pid.PID_ComputeError(error);
  ```

  - 直接传入误差10，计算输出。

------

### 1.3 完整使用示例

```cpp
#include "pid.h"

pid speed_pid(1.0f, 0.1f, 0.05f, 100.0f, 500.0f, 0.01f, 10.0f);
speed_pid.setpoint = 100.0f;  // 目标转速100 RPM

// 假设这是控制循环
float current_rpm = motor.get_rpm();  // 获取电机当前转速
float output = speed_pid.PID_Compute(current_rpm);
motor.set_current(output);  // 设置电机电流
```

------

## 2. `superpid` 类 - 改进版位置式PID控制器

### 2.1 类简介

`superpid` 在经典PID基础上增加了**梯形积分**、**微分先行**和**积分抗饱和**等高级功能，精度更高，响应更平滑。它特别适合对控制精度要求较高的场景，比如电机位置控制或需要动态响应的系统。

------

### 2.2 API函数详解

#### 2.2.1 构造函数

```cpp
superpid(float kp_ = 0.0f, float ki_ = 0.0f, float kd_ = 0.0f, float output_limit_ = 0.0f, float deadzone_ = 0.0f, float integral_separation_threshold_ = 0.0f);
```

- **功能**：创建并初始化`superpid`实例。

- 参数

  ：

  - `kp_`：比例增益。
  - `ki_`：积分增益。
  - `kd_`：微分增益。
  - `output_limit_`：输出限幅值。
  - `deadzone_`：死区阈值。
  - `integral_separation_threshold_`：积分分离阈值。

- **返回值**：无。

- 代码示例

  ：

  ```cpp
  superpid my_pid(1.0f, 0.1f, 0.05f, 1000.0f, 0.01f, 10.0f);
  ```

#### 2.2.2 `superPID_SetParameters`

```cpp
void superPID_SetParameters(float kp, float ki, float kd);
```

- **功能**：动态设置PID增益参数。

- 参数

  ：

  - `kp`：新的比例增益。
  - `ki`：新的积分增益。
  - `kd`：新的微分增益。

- **返回值**：无。

- 代码示例

  ：

  ```cpp
  my_pid.superPID_SetParameters(2.0f, 0.2f, 0.1f);
  ```

#### 2.2.3 `config_all`

```cpp
void config_all(float kp_, float ki_, float kd_, float output_limit_, float deadzone_, float integral_separation_threshold_);
```

- **功能**：一次性配置所有参数。

- 参数

  ：

  - `kp_`：比例增益。
  - `ki_`：积分增益。
  - `kd_`：微分增益。
  - `output_limit_`：输出限幅值。
  - `deadzone_`：死区阈值。
  - `integral_separation_threshold_`：积分分离阈值。

- **返回值**：无。

- 代码示例

  ：

  ```cpp
  my_pid.config_all(2.0f, 0.2f, 0.1f, 1000.0f, 0.01f, 15.0f);
  ```

#### 2.2.4 `superPID_Compute`

```cpp
float superPID_Compute(float input);
```

- **功能**：根据测量值计算PID输出，内部使用时间差计算采样周期。

- 参数

  ：

  - `input`：当前测量值。

- **返回值**：PID输出值。

- 代码示例

  ：

  ```cpp
  float current_pos = 85.0f;
  float output = my_pid.superPID_Compute(current_pos);
  ```

#### 2.2.5 `superPID_ComputeError`

```cpp
float superPID_ComputeError(float error_, float C_V);
```

- **功能**：直接传入误差和当前值计算输出，`C_V`用于微分先行。

- 参数

  ：

  - `error_`：误差值。
  - `C_V`：当前测量值（用于微分计算）。

- **返回值**：PID输出值。

- 代码示例

  ：

  ```cpp
  float error = 5.0f;
  float current_value = 85.0f;
  float output = my_pid.superPID_ComputeError(error, current_value);
  ```

------

### 2.3 完整使用示例

```cpp
#include "superpid.h"

superpid pos_pid(2.0f, 0.2f, 0.1f, 1000.0f, 0.01f, 15.0f);
pos_pid.setpoint = 90.0f;  // 目标位置90度

// 控制循环
float current_pos = motor.get_position();
float output = pos_pid.superPID_Compute(current_pos);
motor.set_current(output);
```

------

## 3. `IncrePID` 类 - 增量式PID控制器

### 3.1 类简介

`IncrePID` 是一个增量式PID，输出的是控制量的增量（而不是绝对值），适合需要平滑控制的场景。它还内置了跟踪微分器（TD）功能（目前被注释），可以平滑目标值，特别适合对突变敏感的电机控制。

------

### 3.2 API函数详解

#### 3.2.1 构造函数

```cpp
IncrePID(float kp_ = 0.0f, float ki_ = 0.0f, float kd_ = 0.0f, float r_ = 0.0f, float output_limit_ = 0.0f, float deadzone_ = 0.0f);
```

- **功能**：创建并初始化`IncrePID`实例。

- 参数

  ：

  - `kp_`：比例增益。
  - `ki_`：积分增益。
  - `kd_`：微分增益。
  - `r_`：TD参数（当前未用）。
  - `output_limit_`：输出限幅值。
  - `deadzone_`：死区阈值。

- **返回值**：无。

- 代码示例

  ：

  ```cpp
  IncrePID my_pid(1.0f, 0.1f, 0.05f, 10.0f, 1000.0f, 0.01f);
  ```

#### 3.2.2 `increPID_SetParameters`

```cpp
void increPID_SetParameters(float kp_, float ki_, float kd_, float r_);
```

- **功能**：设置增益和TD参数。

- 参数

  ：

  - `kp_`：比例增益。
  - `ki_`：积分增益。
  - `kd_`：微分增益。
  - `r_`：TD参数。

- **返回值**：无。

- 代码示例

  ：

  ```cpp
  my_pid.increPID_SetParameters(2.0f, 0.2f, 0.1f, 15.0f);
  ```

#### 3.2.3 `config_all`

```cpp
void config_all(float kp_, float ki_, float kd_, float r_, float output_limit_, float deadzone_);
```

- **功能**：配置所有参数。

- 参数

  ：

  - `kp_`：比例增益。
  - `ki_`：积分增益。
  - `kd_`：微分增益。
  - `r_`：TD参数。
  - `output_limit_`：输出限幅值。
  - `deadzone_`：死区阈值。

- **返回值**：无。

- 代码示例

  ：

  ```cpp
  my_pid.config_all(2.0f, 0.2f, 0.1f, 15.0f, 1000.0f, 0.01f);
  ```

#### 3.2.4 `increPID_setarget`

```cpp
void increPID_setarget(float target_);
```

- **功能**：设置目标值（注意是`setarget`，不是`setpoint`）。

- 参数

  ：

  - `target_`：目标值。

- **返回值**：无。

- 代码示例

  ：

  ```cpp
  my_pid.increPID_setarget(100.0f);
  ```

#### 3.2.5 `increPID_Compute`

```cpp
float increPID_Compute(float input);
```

- **功能**：根据测量值计算增量式PID输出。

- 参数

  ：

  - `input`：当前测量值。

- **返回值**：累积后的输出值。

- 代码示例

  ：

  ```cpp
  float current_rpm = 90.0f;
  float output = my_pid.increPID_Compute(current_rpm);
  ```

#### 3.2.6 `increPID_Computerror`

```cpp
float increPID_Computerror(float error_);
```

- **功能**：直接传入误差计算增量式PID输出（无TD）。

- 参数

  ：

  - `error_`：误差值。

- **返回值**：累积后的输出值。

- 代码示例

  ：

  ```cpp
  float error = 10.0f;
  float output = my_pid.increPID_Computerror(error);
  ```

------

### 3.3 完整使用示例

```cpp
#include "superpid.h"  // 假设IncrePID定义在此头文件中

IncrePID speed_pid(1.5f, 0.1f, 0.05f, 10.0f, 1000.0f, 0.5f);
speed_pid.increPID_setarget(100.0f);  // 目标转速100 RPM

// 控制循环
float current_rpm = motor.get_rpm();
float output = speed_pid.increPID_Compute(current_rpm);
motor.set_current(output);
```

------

## 4. 注意事项

- **单位一致性**：确保`setpoint`和`input`的单位相同（如都用RPM或度）。
- **采样周期**：`superpid`依赖时间差，建议固定周期调用（如每10ms）。
- **TD功能**：`IncrePID`的TD目前被注释，如需启用，请取消`TD()`注释并调整`r`。
- **调试技巧**：查看`p_out`、`i_out`、`d_out`等公共成员变量，分析每个部分的贡献。

------

## 5. 总结

我的PID控制器家族（`pid`、`superpid`、`IncrePID`）提供了丰富的API，覆盖了从参数配置到输出计算的各种需求。希望这份详细的文档能帮你们快速掌握这些类的使用方法，在电机控制或其他场景中游刃有余。有什么问题，随时找我聊哦！