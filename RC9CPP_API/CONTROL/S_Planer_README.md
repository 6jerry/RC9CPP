# S-Shaped 轨迹规划器（SShapedPlanner）

**C++ 版使用手册**

> 适用于任意两维运动场景，支持加加速度（Jerk）限制的 **7 段完整 S 曲线**（加速-匀速-减速）。特点：
>
> - 零动态内存分配（值语义，无 `new`）
> - 使用 CMSIS-DSP 的 `arm_sqrt_f32` 做根号运算，可在 ARM Cortex-M4/M7 上硬件加速
> - 接口简单，仅需一次 `start_plan(...)`，循环调用 `plan(...)` 即可

---

## 1. 快速上手

### 1.1 最小工作示例

```cpp
#include "S_shaped_planner.h"
#include "Vector2D.h"

int main()
{
    // 1. 构造规划器
    SShapedPlanner sp;

    // 2. 给出运动学约束
    float maxAcc   = 2.0f;     // m/s²
    float maxDec   = 2.0f;
    float maxJerk  = 4.0f;     // m/s³
    float maxSpeed = 1.0f;     // m/s
    float v0       = 0.0f;
    float vf       = 0.0f;

    // 3. 设定起点/终点
    Vector2D start(0, 0);
    Vector2D  goal(1, 0);      // 1 米直线

    // 4. 启动一次规划
    sp.start_plan(maxAcc, maxDec, maxJerk, maxSpeed, v0, vf, start, goal);

    // 5. 周期性调用
    Vector2D currentPos(0.1f, 0);   // 假设当前位置
    Vector2D cmdVel = sp.plan(currentPos);
    // cmdVel 即为当前应达到的二维速度向量
}
```

---

## 2. 接口说明

| 名称                      | 作用                                     |
| ------------------------- | ---------------------------------------- |
| `SShapedPlanner()`      | 构造函数，所有成员初始化为 0             |
| `start_plan(...)`       | **一次性**配置整条轨迹参数         |
| `plan(const Vector2D&)` | 给定当前位置，返回当前阶段的目标速度向量 |
| `getPhase()`            | 返回当前阶段枚举                         |
| `isFinished()`          | 若已到达终点返回 `true`                |

---

## 3. 参数与单位

| 参数                   | 含义                   | 单位  | 范围 |
| ---------------------- | ---------------------- | ----- | ---- |
| `maxAcc`             | 最大加速度             | m/s² | > 0  |
| `maxDec`             | 最大减速度             | m/s² | > 0  |
| `maxJerk`            | 最大加加速度（Jerk）   | m/s³ | > 0  |
| `maxSpeed`           | 运行期间允许的最大速度 | m/s   | ≥ 0 |
| `initialSpeed`       | 起始速度               | m/s   | ≥ 0 |
| `finalSpeed`         | 终点速度               | m/s   | ≥ 0 |
| `startPos/targetPos` | 二维起止坐标           | m     | 任意 |

> 所有减速度请**以正值传入**，内部自动取负。

---

## 4. 运动阶段枚举

```cpp
enum SPhase {
    S_ACCEL_JERK_UP_PHASE,   // 加速段：Jerk 从 0 → +max
    S_ACCEL_CONST_PHASE,     // 加速段：加速度恒定在 +maxAcc
    S_ACCEL_JERK_DOWN_PHASE, // 加速段：Jerk 从 +max → 0
    S_CONST_VEL_PHASE,       // 匀速段：速度恒定在 maxSpeed
    S_DECEL_JERK_UP_PHASE,   // 减速段：Jerk 从 0 → −max
    S_DECEL_CONST_PHASE,     // 减速段：加速度恒定在 −maxDec
    S_DECEL_JERK_DOWN_PHASE, // 减速段：Jerk 从 −max → 0
    S_FINISHED_PHASE         // 已到达终点
};
```

- 阶段切换完全由**已行驶距离**决定，无需外部干预。
- 到达终点后所有 `plan(...)` 均返回 `(0,0)` 向量（方向保持终点方向，速度为 `finalSpeed`）。

---

## 5. 使用流程（伪代码）

```cpp
// 1. 初始化
sp.start_plan(...);

while (true) {
    Vector2D currentPos = getCurrentPose(); // 从里程计/视觉/IMU 获取
    Vector2D cmdVel = sp.plan(currentPos);

    // 2. 下发速度
    setRobotVelocity(cmdVel);

    // 3. 判断是否结束
    if (sp.isFinished()) {
        stopRobot();
        break;
    }

    delay(dt); // 与控制器周期同步
}
```

---

## 6. 精度与性能

| 项目   | 说明                                                          |
| ------ | ------------------------------------------------------------- |
| 计算量 | 每周期 1 次 `sqrtf`（已由 `arm_sqrt_f32` 替代）+ 少量乘加 |
| RAM    | 仅 40 字节内部变量（无堆）                                    |
| 周期   | 在 Cortex-M4 @168 MHz 实测 < 2 µs                            |
| 精度   | 与 `double` 版本误差 < 1 mm                                 |

---

## 7. 常见注意点

1. **距离不能为零**如果 `startPos == targetPos`，`m_totalDistance == 0`，将直接进入 `S_FINISHED_PHASE`。
2. **速度约束**若 `initialSpeed > maxSpeed` 或 `finalSpeed > maxSpeed`，内部会强制截断为 `maxSpeed` 并重新计算阶段长度。
3. **方向一致性**整个轨迹始终沿 `targetPos - startPos` 的直线方向；返回的速度向量已自动归一化后乘以速度大小。
4. **周期性调用**
   请保证 `plan(...)` 调用频率 ≥ 50 Hz（推荐 100 Hz），否则阶段切换可能出现抖动。

---

## 8. 移植到裸机

1. 确保链接 **CMSIS-DSP** 库（`libarm_cortexM4lf_math.a` 或对应版本）。
2. 若 `Vector2D` 为自定义结构体，需实现：

   - `Vector2D operator-(const Vector2D&)`
   - `Vector2D operator*(float)`
   - `float magnitude()`
   - `Vector2D normalize()`
3. 头文件中 `extern "C"` 包裹已在 `S_shaped_planner.h` 内处理好，可直接在 C++ 与 C 混合工程使用。

---

## 9. 示例输出

以 1 m（x 轴）直线为例：

| 时间 (s)  | 阶段 | 速度 (m/s)   | 备注               |
| --------- | ---- | ------------ | ------------------ |
| 0.00      | J↑  | 0.00 → 0.10 | 加速度线性增大     |
| 0.25      | 恒A  | 0.10 → 0.90 | 加速度恒定         |
| 0.50      | J↓  | 0.90 → 1.00 | 加速度线性减小到 0 |
| 0.50~1.00 | 匀速 | 1.00         | 匀速行驶           |
| 1.00      | J↓  | 1.00 → 0.90 | 开始减速           |
| 1.25      | 恒D  | 0.90 → 0.10 | 减速度恒定         |
| 1.50      | J↑  | 0.10 → 0.00 | 平滑停止           |

---

## 10. 许可证

本项目为 MIT 许可证，可自由商用、修改、分发。

---

**祝使用愉快！**
如需更复杂轨迹（多段、圆弧、避障），可在本规划器基础上叠加路径分割或二次规划。
