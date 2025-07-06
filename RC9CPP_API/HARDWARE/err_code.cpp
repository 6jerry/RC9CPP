#include "err_code.h"

// 定义系统中最多支持注册的 error_check 实例数量
#define MAX_INSTANCES 32

// 全局错误实例数组，用于集中管理所有注册的 error_check 对象
static error_check *err_devices[MAX_INSTANCES] = {nullptr};

// 当前已注册的实例数量（用于分配索引）
static uint8_t instance_num = 0;

// 当前活跃的实例数量（用于遍历时使用）
static uint8_t now_instance_num = 0;

/**
 * @brief error_check 类的构造函数
 *
 * 调用 init() 方法进行初始化，确保每个子类都能正确设置设备 ID 并注册自己。
 */
error_check::error_check() {
    init();
}

/**
 * @brief 初始化方法
 *
 * 设置设备唯一标识符并调用 registerInstance 注册当前对象。
 */
void error_check::init() {
    err_id = set_device_id();       // 子类必须实现 set_device_id()
    registerInstance(this);         // 将当前对象注册到全局列表
}

/**
 * @brief 注册当前 error_check 实例到全局数组中
 * @param instance 指向当前 error_check 实例的指针
 *
 * 此函数负责将继承自 error_check 的对象加入全局管理数组，
 * 同时更新注册计数器和当前活跃计数器。
 */
void error_check::registerInstance(error_check *instance) {
    if (instance_num <= MAX_INSTANCES) {
        err_devices[instance_num++] = instance;   // 增加注册索引
        now_instance_num++;                       // 增加活跃实例数
    }
}

/**
 * @brief 处理所有注册的 error_check 实例的错误检测结果
 *
 * 遍历所有注册过的 error_check 对象，调用 check_error() 方法获取错误码，
 * 若返回值不是 ERR_CODE_WORK_SUCCESS，则通过 sendByteData 发送错误信息。
 */
void error_checker::process_data() {
    static uint8_t communicate_id = 7;      // 设定发送数据的目标通信 ID（示例值）
    static uint8_t data[2] = {0};           // 缓冲区，分别保存设备 ID 和错误码

    for (int i = 0; i < now_instance_num; i++) {
        // 获取当前设备的错误码
        err_code tmp_code = err_devices[i]->check_error();

        // 如果有错误发生（非成功状态）
        if(tmp_code != ERR_CODE_WORK_SUCCESS){
            data[0] = err_devices[i]->err_id;   // 第一个字节为设备 ID
            data[1] = tmp_code;                 // 第二个字节为错误码
            sendByteData(communicate_id, data, 2); // 发送错误信息
        }
    }
}