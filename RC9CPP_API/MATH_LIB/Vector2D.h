#ifndef VECTOR2D_H
#define VECTOR2D_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <arm_math.h>

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus
#define QUEUE_CAPACITY 3 // 队列最大容量
class Vector2D
{
public:
    float32_t x;
    float32_t y;

    // 构造函数
    Vector2D();
    Vector2D(float32_t x_, float32_t y_);

    // 赋值运算符
    Vector2D &operator=(const Vector2D &other);

    // 向量加法
    Vector2D operator+(const Vector2D &other) const;

    // 向量减法
    Vector2D operator-(const Vector2D &other) const;
    Vector2D operator-() const;

    // 向量点乘
    float32_t operator*(const Vector2D &other) const;

    // 向量乘以标量
    Vector2D operator*(float32_t scalar) const;

    // 标量乘以向量 (标量在左侧)
    friend Vector2D operator*(float32_t scalar, const Vector2D &vec);

    // 向量的模（长度）
    float32_t magnitude() const;

    // 单位化向量
    Vector2D normalize() const;

    // 向量投影：投影this向量到other向量上
    Vector2D project_onto(const Vector2D &other) const;
};

class Vector2DQueue
{
private:
    Vector2D data[QUEUE_CAPACITY]; // 用于存储队列元素的数组
    int front;                     // 队首索引
    int rear;                      // 队尾索引
    int size;                      // 当前队列大小

public:
    // 构造函数
    Vector2DQueue();

    // 检查队列是否为空
    bool isEmpty() const;

    // 检查队列是否已满
    bool isFull() const;

    // 返回队列中的元素数量
    int queueSize() const;

    // 入队操作
    bool enqueue(const Vector2D &vec);

    // 强制入队操作（覆盖队尾元素）
    void forceEnqueue(const Vector2D &vec);

    // 出队操作
    bool dequeue(Vector2D &vec);

    // 查看队首元素
    Vector2D peek() const;

    // 将一个数组压入队列，数组索引小的元素先压入
    bool enqueueArray(const Vector2D arr[], int length);

    void forceEnqueueArray(const Vector2D arr[], int length);

    // 清空队列
    void clear();

    float totalDistance() const;
};

#endif // VECTOR2D_H
#endif
