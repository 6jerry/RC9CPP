#include "Vector2D.h"
// try12
//  默认构造函数
Vector2D::Vector2D() : x(0.0f), y(0.0f) {}

// 带参数构造函数
Vector2D::Vector2D(float32_t x_, float32_t y_) : x(x_), y(y_) {}

// 重载赋值运算符
Vector2D &Vector2D::operator=(const Vector2D &other)
{
    if (this != &other)
    {
        this->x = other.x;
        this->y = other.y;
    }
    return *this;
}

// 向量加法
Vector2D Vector2D::operator+(const Vector2D &other) const
{
    return Vector2D(this->x + other.x, this->y + other.y);
}

// 向量减法
Vector2D Vector2D::operator-(const Vector2D &other) const
{
    return Vector2D(this->x - other.x, this->y - other.y);
}

// 向量点乘，使用DSP库
float32_t Vector2D::operator*(const Vector2D &other) const
{
    float32_t dotProduct;
    float32_t vecA[2] = {this->x, this->y};
    float32_t vecB[2] = {other.x, other.y};
    arm_dot_prod_f32(vecA, vecB, 2, &dotProduct);
    return dotProduct;
}

// 向量乘以标量
Vector2D Vector2D::operator*(float32_t scalar) const
{
    return Vector2D(this->x * scalar, this->y * scalar);
}

// 标量乘以向量，标量在左侧
Vector2D operator*(float32_t scalar, const Vector2D &vec)
{
    return Vector2D(vec.x * scalar, vec.y * scalar);
}

// 向量的模
float32_t Vector2D::magnitude() const
{
    float32_t result;
    arm_sqrt_f32((this->x * this->x) + (this->y * this->y), &result);
    return result;
}

// 单位化向量
Vector2D Vector2D::normalize() const
{
    Vector2D result;
    float32_t mag = this->magnitude();
    if (mag > 0)
    {
        result.x = this->x / mag;
        result.y = this->y / mag;
    }
    return result;
}

// 向量投影
Vector2D Vector2D::project_onto(const Vector2D &other) const
{
    float32_t dotProduct = (*this) * other;
    float32_t magOtherSquared = other.x * other.x + other.y * other.y;
    float32_t scalar = dotProduct / magOtherSquared;
    return other * scalar;
}
// 负号运算符重载
Vector2D Vector2D::operator-() const
{
    return Vector2D(-this->x, -this->y);
}

Vector2DQueue::Vector2DQueue() : front(0), rear(-1), size(0) {}

// 检查队列是否为空
bool Vector2DQueue::isEmpty() const
{
    return size == 0;
}

// 检查队列是否已满
bool Vector2DQueue::isFull() const
{
    return size == QUEUE_CAPACITY;
}

// 返回队列中的元素数量
int Vector2DQueue::queueSize() const
{
    return size;
}

// 入队操作
bool Vector2DQueue::enqueue(const Vector2D &vec)
{
    if (isFull())
    {
        return false; // 队列已满，入队失败
    }
    rear = (rear + 1) % QUEUE_CAPACITY; // 循环队列
    data[rear] = vec;                   // 插入元素
    size++;
    return true; // 入队成功
}

// 出队操作
bool Vector2DQueue::dequeue(Vector2D &vec)
{
    if (isEmpty())
    {
        return false; // 队列为空，出队失败
    }
    vec = data[front];                    // 获取队首元素
    front = (front + 1) % QUEUE_CAPACITY; // 更新队首索引
    size--;
    return true; // 出队成功
}

// 查看队首元素
Vector2D Vector2DQueue::peek() const
{
    if (isEmpty())
    {
        return Vector2D(0.0f, 0.0f); // 队列为空，查看失败
    }

    return data[front]; // 查看成功
}

// 强制入队操作（覆盖队尾元素）
void Vector2DQueue::forceEnqueue(const Vector2D &vec)
{
    if (isFull())
    {
        front = (front + 1) % QUEUE_CAPACITY; // 移动队首索引以覆盖最早插入的元素
        size--;                               // 因覆盖，队列大小减少
    }
    rear = (rear + 1) % QUEUE_CAPACITY; // 插入新元素
    data[rear] = vec;
    size++; // 更新队列大小
}

// 将一个数组压入队列，数组索引小的元素先压入
bool Vector2DQueue::enqueueArray(const Vector2D arr[], int length)
{
    if (length <= 0)
    {
        return false; // 无效长度，直接返回失败
    }

    for (int i = 0; i < length; ++i)
    {
        // 检查是否能够正常入队
        if (!enqueue(arr[i]))
        {
            return false; // 如果有元素无法入队，返回失败
        }
    }
    return true; // 数组全部成功入队
}

// 将一个数组压入队列，数组索引小的元素先压入
void Vector2DQueue::forceEnqueueArray(const Vector2D arr[], int length)
{
    for (int i = 0; i < length; ++i)
    {
        forceEnqueue(arr[i]); // 强制入队覆盖队尾元素
    }
}

// 清空队列
void Vector2DQueue::clear()
{
    front = 0;
    rear = -1;
    size = 0;
}

float Vector2DQueue::totalDistance() const
{
    float total = 0.0f;
    if (size < 2)
        return total;
    int idx = front;
    Vector2D prev = data[idx];
    for (int i = 1; i < size; i++)
    {
        idx = (front + i) % QUEUE_CAPACITY;
        total += (data[idx] - prev).magnitude();
        prev = data[idx];
    }
    return total;
}
