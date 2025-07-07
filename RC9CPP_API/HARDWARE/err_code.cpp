#include "err_code.h"

// 定义系统中最多支持注册的同类 error_check 实例数量
#define SIMILAR_MAX_INSTANCES 16

// 全局实例数组，用于集中管理所有注册的 error_check 对象
static error_check *err_devices[DEVICE_CLASS_NUM][MAX_INSTANCES] = {nullptr};
static error_check *err_devices_tmp[DEVICE_CLASS_NUM * MAX_INSTANCES] = {nullptr};
// 区分同类设备的id（不同的实例）
static uint8_t arr_similar_ids[DEVICE_CLASS_NUM] = {0};

// 当前已注册的同类实例数量（用于分配索引）
static uint8_t instance_num[DEVICE_CLASS_NUM] = {0};

static uint8_t now_instance = 0;

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
	err_devices_tmp[now_instance++] = instance;
}


err_code error_check::check_timeout(){
	static uint8_t count = 0;
	if(!timeout_flag){
		count = 0;
		timeout_flag = true;
		return ERR_CODE_WORK_SUCCESS;
	}else{
		if(count++ > 15){
			timeout_flag = true;
			return ERR_CODE_TIMEOUT;
		}else{
			return ERR_CODE_WORK_SUCCESS;
		}
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
    static uint8_t data[3] = {0};           // 缓冲区，分别保存设备类型 ID 和错误码
	static bool start_flag = false;
	if(!start_flag){
		for(int i = 0; i < now_instance; i++){
			if (instance_num[err_devices_tmp[i]->err_id] < SIMILAR_MAX_INSTANCES) {
				err_devices_tmp[i]->instance_id = arr_similar_ids[err_devices_tmp[i]->err_id];  // 同类的id自增
				arr_similar_ids[err_devices_tmp[i]->err_id]++;
				err_devices[err_devices_tmp[i]->err_id][instance_num[err_devices_tmp[i]->err_id]++]  \
				= err_devices_tmp[i];   // 增加注册索引
			}
		}
		start_flag = true;
	}

    for (int i = 0; i < DEVICE_CLASS_NUM; i++) {
        for (int j = 0; j < instance_num[i]; j++) {
            // 获取当前设备的错误码
            err_code tmp_code = err_devices[i][j]->check_error();
            // 如果有错误发生（非成功状态）
            if (tmp_code != ERR_CODE_WORK_SUCCESS) {
                data[0] = err_devices[i][j]->err_id;   // 第一个字节为设备类型 ID
                data[1] = err_devices[i][j]->instance_id;            // 第二个字节为同类设备区别ID
                data[2] = tmp_code;                 // 第三个字节为错误码
                sendByteData(communicate_id, data, 3); // 发送错误信息
            }
			err_code timeout_tmp_check = err_devices[i][j]->check_timeout();
			if(timeout_tmp_check == ERR_CODE_TIMEOUT){
				data[0] = err_devices[i][j]->err_id;   // 第一个字节为设备类型 ID
                data[1] = err_devices[i][j]->instance_id;            // 第二个字节为同类设备区别ID
                data[2] = timeout_tmp_check;                 // 第三个字节为错误码
                sendByteData(communicate_id, data, 3); // 发送错误信息
			}
        }
    }
}