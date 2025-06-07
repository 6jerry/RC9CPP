#include "ros_sensor.h"
#include "Vector2D.h"

#define PI 3.141592653589793f
// 0值去除(传过来的float一定不为0)
static void zero_removal(float *input, const float last_input, const float la_lastinput)
{
	*input = (fabsf(*input) <= 1e-4) ? last_input + (last_input - la_lastinput) : *input;
}

ros_sensor::ros_sensor()
{
	KalmanFilter_init(0.01f, 0.1f, 0.1f);
}

void ros_sensor::DataReceivedCallback(const uint8_t *byteData, const float *floatData, uint8_t id, uint16_t byteCount)
{

	// 上一次变量存储
	static float previous_vertial_plane_deviation_x = 0.0f;
	static float previous_vertial_plane_deviation_y = 0.0f;
	static float previous_world_pos_x = 0.0f;
	static float previous_world_pos_y = 0.0f;
	static float previous_yaw_angle = 0.0f;
	static float pe_previous_world_pos_x = 0.0f;
	static float pe_previous_world_pos_y = 0.0f;
	static float pe_previous_yaw_angle = 0.0f;

	//处理相机数据
	if(id == 1 && byteCount == 8){
		camera_info.vertial_plane_deviation.x = floatData[0];
		camera_info.vertial_plane_deviation.y = floatData[1];
	}
	//处理雷达数据
	else if (id == 2 && byteCount == 20)
	{	
		ros_radar_loaction.world_pos.x = -(floatData[1]);
		ros_radar_loaction.world_pos.y = (floatData[0]); // 把上位机坐标与追踪坐标方向对齐
		ros_radar_loaction.yaw_angle = -filter(floatData[2]);
		//		if(fabsf(floatData[2]) < 0.02f && fabsf(floatData[3]) < 0.05f){
		//			map_origin_init_flag = false; //重置映射原点
		//		}
	}
	else{
		return;
	}
	// 记录上上次数据
	pe_previous_world_pos_x = previous_world_pos_x;
	pe_previous_world_pos_y = previous_world_pos_y;
	pe_previous_yaw_angle = previous_yaw_angle;
	// 去除雷达异常值
	zero_removal(&ros_radar_loaction.world_pos.x, previous_world_pos_x, pe_previous_world_pos_x);
	zero_removal(&ros_radar_loaction.world_pos.y, previous_world_pos_y, pe_previous_world_pos_y);
	zero_removal(&ros_radar_loaction.yaw_angle, previous_yaw_angle, pe_previous_yaw_angle);
	// 阈值限制
	static float max_change = 80.0f; // 像素
	if (fabsf(camera_info.vertial_plane_deviation.x - previous_vertial_plane_deviation_x) > max_change)
	{
		camera_info.vertial_plane_deviation.x = previous_vertial_plane_deviation_x +
												((camera_info.vertial_plane_deviation.x - previous_vertial_plane_deviation_x) > 0 ? 1 : -1) * max_change / 4;
	}
	if (fabsf(camera_info.vertial_plane_deviation.y - previous_vertial_plane_deviation_y) > max_change)
	{
		camera_info.vertial_plane_deviation.y = previous_vertial_plane_deviation_y +
												((camera_info.vertial_plane_deviation.y - previous_vertial_plane_deviation_y) > 0 ? 1 : -1) * max_change / 4;
	}
	previous_vertial_plane_deviation_x = camera_info.vertial_plane_deviation.x;
	previous_vertial_plane_deviation_y = camera_info.vertial_plane_deviation.y;
	static float max_change_ = 3.0f; // m
	if (fabsf(ros_radar_loaction.world_pos.x - previous_world_pos_x) > max_change_)
	{
		ros_radar_loaction.world_pos.x = previous_world_pos_x +
										 ((ros_radar_loaction.world_pos.x - previous_world_pos_x) > 0 ? 1 : -1) * max_change_ / 4;
	}
	if (fabsf(ros_radar_loaction.world_pos.y - previous_world_pos_y) > max_change_)
	{
		ros_radar_loaction.world_pos.y = previous_world_pos_y +
										 ((ros_radar_loaction.world_pos.y - previous_world_pos_y) > 0 ? 1 : -1) * max_change_ / 4;
	}
	if (fabsf(ros_radar_loaction.yaw_angle - previous_yaw_angle) > max_change_ * 121.0f)
	{
		ros_radar_loaction.yaw_angle = previous_yaw_angle + ((ros_radar_loaction.yaw_angle - previous_yaw_angle) > 0 ? 1 : -1) * max_change_ * 0.5f;
	}

	// 异常值标志位判断
	if (fabsf(ros_radar_loaction.world_pos.x) < 2e-6 && fabsf(ros_radar_loaction.world_pos.y) < 2e-6)
	{
		get_zero_flag = true;
	}
	if (previous_world_pos_x == ros_radar_loaction.world_pos.x || previous_world_pos_y == ros_radar_loaction.world_pos.y)
	{
		get_zero_flag = true;
	}


	// 差分原点标定
	if (!tf_.map_origin_init_flag)
	{
		tf_.coordinate_map(&ros_radar_loaction.world_pos, &real_radar_world_pos,my_r, my_rad);
		tf_.map_origin = real_radar_world_pos;
		tf_.map_origin_init_flag = true;
		// 差分定位
		tf_.localize_with_diff(&real_radar_world_pos);
	}

	// 自旋映射
	if (!get_zero_flag)
	{ // 对于异常值不进行映射

		tf_.coordinate_map(&ros_radar_loaction.world_pos, &real_radar_world_pos, my_r, my_rad - imu_->get_yaw_rad());
		// 差分定位
		tf_.localize_with_diff(&real_radar_world_pos);
		// 发送校准信息
		imu_->imu_relocate(real_radar_world_pos.x, -real_radar_world_pos.y, 0);
	}
	else
	{ // 重置0值标志位
		get_zero_flag = false;
	}
	// 更新上次变量
	previous_world_pos_x = ros_radar_loaction.world_pos.x;
	previous_world_pos_y = ros_radar_loaction.world_pos.y;
	previous_yaw_angle = ros_radar_loaction.yaw_angle;
	/*****************发给雷达的占位数据**************/
	sendFloatData(2, &previous_world_pos_x, 1);
	/***********************************************/
	if (relocate_flag && imu_->get_world_vel_x()  < 0.5f && imu_->get_world_vel_y() < 0.5f)
	{ // 检测标志位来判断是否校准action
		imu_->imu_relocate(real_radar_world_pos.x, -real_radar_world_pos.y, 0);
		return;
	}
	
	if(rst_radar){
		rst_radar = false;
		imu_rst();
	}
}

Vector2D ros_sensor::get_world_pos(void)
{
	return real_radar_world_pos;
}

float ros_sensor::get_heading(void)
{
	return ros_radar_loaction.yaw_angle;
}

float ros_sensor::get_yaw_rad()
{
	return ros_radar_loaction.yaw_angle * 0.01745329252f;
}

// 添加重定位imu
void ros_sensor::add_recolate_imu(imu *_imu)
{
	imu_ = _imu;
}

// 设置重定位标志，用雷达校准
void ros_sensor::relocate_imu(void)
{
	relocate_flag = true;
}

// 设置重定位标志，停止用雷达校准
void ros_sensor::stop_relocate(void)
{
	relocate_flag = false;
}

void ros_sensor::imu_rst()
{
	//重置雷达同时开始定位
	float arr[3] = {0.0f, 0.0f, 0.0f}; // x, y ,angle
	sendFloatData(1, arr, 3);
	tf_.map_origin_init_flag = false;
}