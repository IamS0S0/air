#include <cmath>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <iostream>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandLong.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <string>
#include <tf/transform_listener.h>
#include <vector>
/*新的识别*/
#include <cv_bridge/cv_bridge.h>
#include <opencv2/objdetect.hpp>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/Image.h>

using namespace std;

#define ALTITUDE 1.3f

mavros_msgs::PositionTarget setpoint_raw;
/************************************************************************
函数声明
*************************************************************************/
void state_cb(const mavros_msgs::State::ConstPtr &msg);
void local_pos_cb(const nav_msgs::Odometry::ConstPtr &msg);
void image_cb(const sensor_msgs::ImageConstPtr &msg);
void lidar_cb(const sensor_msgs::LaserScan::ConstPtr &scan);
void init_location(double &init_x, double &init_y);
void cal_min_distance();
void cal_best_angle();

bool mission_pos_cruise(float x, float y, float z, float yaw, float error_max);
bool precision_land();
bool collision_avoidance(float x, float y, float z, float err_max, double max_vel = 0.8);
bool cross_ring(double x, double y, double err_max);
bool move_in_drone_coordinate(double x, double y, double err_max);
bool collision_in_drone_coordinate(double x, double y, double err_max, double max_vel);
bool is_exist_ring();

int isobs(double x, double y, double total);
int cal_middle(double *front, int max_start, int max_end);

float satfunc(float data, float Max);

double call_mid_len(double angle1, double angle2);
double call_len(double angle1, double angle2);
double cal_y(double angle);
double cal_x(double angle);


double best_angle;
double distance_c = 9999;
double angle_c;
float R_outside = 3, R_inside = 1; // 安全半径 [避障算法相关参数]
float distance_cx, distance_cy;	   // 最近障碍物距离XY
sensor_msgs::LaserScan Laser;	   // 激光雷达点云数据
sensor_msgs::LaserScan Laser_std;  // 激光雷达点云数据
bool cross_ring_flag = false;
bool find_ring = false;
bool isinit = false;
double nearest_ring[2];
int mode = 1;
double init_x = 0, init_y = 0;
/************************************************************************
类声明 (stick类为细长型柱子)(ring类为环)
*************************************************************************/
typedef struct Angle
{
	double start, end;
	Angle(double _start, double _end) : start(_start), end(_end) {}
} Angle;

typedef struct AddPos
{
	double x = 0, y = 0, d;
	Angle R;
	AddPos(double _start, double _end, double _x, double _y, double _d) : R(_start, _end), x(_x), y(_y), d(_d) {}
} AddPos;

class stick
{
private:
	double stick_width;
	double std_stick_err;

public:
	std::vector<Angle> stick_angle; // 储存角度下标<start_angle,end_angle>
	stick(double _stick_width, double _std_stick_err) : stick_width(_stick_width),
														std_stick_err(_std_stick_err) {};

	~stick() = default;
	void FindStick();
};

class ring : public stick
{
private:
	double inner_width;
	double std_ring_err;

public:
	std::vector<AddPos> ring_ifo; // 储存环的所有信息
	ring(double _stick_width, double _std_stick_err, double _inner_width, double _std_ring_err)
		: stick(_stick_width, _std_stick_err),
		  inner_width(_inner_width),
		  std_ring_err(_std_ring_err) {};
	~ring() {};

	bool IsRing();
	void NearestRing(); // 理应只返回偏移量x,y 但是懒得再额外写一个结构体 并且其他数据可能会发挥作用 故一块返回了
};

ring all_ring(0.1, 0.1, 1.0, 0.3);
/************************************************************************
类实现
*************************************************************************/
/************************************************************************
成员函数 1：寻找stick
*************************************************************************/
void stick::FindStick()
{
	double start = 90, end = 0, len = 0;
	int counting = 0;
	for (int i = 90; i < 270; i++)
	{
		if (counting)
		{
			if (Laser_std.ranges[i] < 5.8 && abs(Laser_std.ranges[i] - Laser_std.ranges[i - 1]) < 0.15)
				continue;
			else
			{
				end = i - 1;
				counting = 0;
				if (end - start >= 1)
				{
					len = call_len(start, end);
					if (len < stick_width + std_stick_err && len > stick_width - std_stick_err)
						stick_angle.push_back(Angle(start, end));
					ROS_INFO("添加障碍物(%f,%f)，宽度%f", start, end, len);
				}
			}
		}
		else
		{
			if (Laser_std.ranges[i] < 5.8)
			{
				start = i;
				counting = 1;
			}
			else
				continue;
		}
	}
	// 处理循环结束后仍在统计的尾部区间
	if (counting)
	{
		end = 269; // 尾部区间的结束角是最后一个角度（269）
		len = call_len(start, end);
		if (len < stick_width + std_stick_err && len > stick_width - std_stick_err)
			stick_angle.push_back(Angle(start, end));
	}
}
/************************************************************************
成员函数 2：判断圆环是否存在
*************************************************************************/
bool ring::IsRing()
{
	double len;
	bool exist = false;
	ring_ifo.clear();
	for (auto R_one = stick_angle.begin(); R_one != stick_angle.end(); ++R_one)
	{
		for (auto L_one = R_one + 1; L_one != stick_angle.end(); ++L_one)
		{
			len = call_len(R_one->end, L_one->start);
			if (len < inner_width + std_ring_err && len > inner_width - std_ring_err)
			{
				ROS_WARN("障碍物距离 %f m", len);
				// 将圆环左右边界写入，中心关于无人机的偏移量x和y先不计算，等返回时计算。AddPos(_start, _end,_x,_y,_d)
				ring_ifo.push_back(AddPos(R_one->end, L_one->start, 0, 0, call_mid_len(R_one->end, L_one->start)));
				exist = true;
			}
		}
	}
	return exist;
}
/************************************************************************
成员函数 3：找到最近圆环
*************************************************************************/
void ring::NearestRing()
{
	auto min_i = ring_ifo.begin();
	for (auto it = ring_ifo.begin() + 1; it != ring_ifo.end(); ++it)
	{
		if (it->d < min_i->d)
			min_i = it;
	}

	// min_i->x=(-cal_x(min_i->R.start-90)+cal_x(min_i->R.end-90))/2;
	// min_i->y=(cal_y(min_i->R.start-90)+cal_y(min_i->R.end-90))/2;
	// return *min_i;

	ROS_INFO("min_i->R=(%f + %f)", min_i->R.start, min_i->R.end);
	ROS_INFO("nearest_ring[0]=(%f + %f) /2 ", cal_x(min_i->R.start - 90), cal_x(min_i->R.end - 90));
	ROS_INFO("nearest_ring[1]=(%f + %f) /2 ", cal_y(min_i->R.start - 90), cal_y(min_i->R.end - 90));
	nearest_ring[0] = (cal_x(min_i->R.start - 90) + cal_x(min_i->R.end - 90)) / 2;
	nearest_ring[1] = (cal_y(min_i->R.start - 90) + cal_y(min_i->R.end - 90)) / 2;
}

/************************************************************************
函数 1：无人机状态回调函数
*************************************************************************/
mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr &msg)
{
	current_state = *msg;
}

/************************************************************************
函数 2：回调函数接收无人机的里程计信息
从里程计信息中提取无人机的位置信息和姿态信息
*************************************************************************/
tf::Quaternion quat;
nav_msgs::Odometry local_pos;
double roll, pitch, yaw;
float init_position_x_take_off = 0;
float init_position_y_take_off = 0;
float init_position_z_take_off = 0;
float init_yaw_take_off = 0;
bool flag_init_position = false;

void local_pos_cb(const nav_msgs::Odometry::ConstPtr &msg)
{
	local_pos = *msg;
	tf::quaternionMsgToTF(local_pos.pose.pose.orientation, quat);
	tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
	if (flag_init_position == false && (local_pos.pose.pose.position.z != 0))
	{
		init_position_x_take_off = local_pos.pose.pose.position.x;
		init_position_y_take_off = local_pos.pose.pose.position.y;
		init_position_z_take_off = local_pos.pose.pose.position.z;
		init_yaw_take_off = yaw;
		flag_init_position = true;
	}
}

/************************************************************************
函数 3: 无人机位置控制
控制无人机飞向（x, y, z）位置，yaw为目标航向角，error_max为允许的误差范围
进入函数后开始控制无人机飞向目标点，返回值为bool型，表示是否到达目标点
*************************************************************************/
float mission_pos_cruise_last_position_x = 0;
float mission_pos_cruise_last_position_y = 0;
bool mission_pos_cruise_flag = false;
bool mission_pos_cruise(float x, float y, float z, float yaw, float error_max)
{
	if (mission_pos_cruise_flag == false)
	{
		mission_pos_cruise_last_position_x = local_pos.pose.pose.position.x;
		mission_pos_cruise_last_position_y = local_pos.pose.pose.position.y;
		mission_pos_cruise_flag = true;
	}
	setpoint_raw.type_mask = /*1 + 2 + 4 */ +8 + 16 + 32 + 64 + 128 + 256 + 512 /*+ 1024 */ + 2048;
	setpoint_raw.coordinate_frame = 1;
	setpoint_raw.position.x = x + init_position_x_take_off;
	setpoint_raw.position.y = y + init_position_y_take_off;
	setpoint_raw.position.z = z + init_position_z_take_off;
	setpoint_raw.yaw = yaw;
	ROS_INFO("当前位置(%.2f,%.2f,%.2f) | 目标位置(%.2f,%.2f,%.2f)",
			 local_pos.pose.pose.position.x,
			 local_pos.pose.pose.position.y,
			 local_pos.pose.pose.position.z,
			 x,
			 y,
			 z);
	/*ROS_INFO("now (%.2f,%.2f,%.2f,%.2f) to ( %.2f, %.2f, %.2f, %.2f)", local_pos.pose.pose.position.x, local_pos.pose.pose.position.y, local_pos.pose.pose.position.z, yaw * 180.0 / M_PI, x + init_position_x_take_off, y + init_position_y_take_off, z + init_position_z_take_off, yaw * 180.0 / M_PI);*/
	if (fabs(local_pos.pose.pose.position.x - x - init_position_x_take_off) < 0.05 && fabs(local_pos.pose.pose.position.y - y - init_position_y_take_off) < 0.05 && fabs(local_pos.pose.pose.position.z - z) < 0.06 && fabs(yaw - yaw) < 0.05)
	{
		ROS_INFO("到达目标点，巡航点任务完成");
		mission_pos_cruise_flag = false;
		return true;
	}
	return false;
}

/************************************************************************
函数 4:降落
无人机当前位置作为降落点，缓慢下降至地面
返回值为bool型，表示是否降落完成
*************************************************************************/
float precision_land_init_position_x = 0;
float precision_land_init_position_y = 0;
bool precision_land_init_position_flag = false;
ros::Time precision_land_last_time;
bool precision_land()
{
	if (!precision_land_init_position_flag)
	{
		precision_land_init_position_x = local_pos.pose.pose.position.x;
		precision_land_init_position_y = local_pos.pose.pose.position.y;
		precision_land_last_time = ros::Time::now();
		precision_land_init_position_flag = true;
	}
	setpoint_raw.position.x = precision_land_init_position_x;
	setpoint_raw.position.y = precision_land_init_position_y;
	setpoint_raw.position.z = -0.15;
	setpoint_raw.type_mask = /*1 + 2 + 4 + 8 + 16 + 32*/ +64 + 128 + 256 + 512 /*+ 1024 + 2048*/;
	setpoint_raw.coordinate_frame = 1;
	if (ros::Time::now() - precision_land_last_time > ros::Duration(5.0))
	{
		ROS_INFO("Precision landing complete.");
		precision_land_init_position_flag = false; // Reset for next landing
		return true;
	}
	return false;
}
/************************************************************************
函数 5:传感器回调函数
控制无人机飞向（x, y, z）位置，yaw为目标航向角，error_max为允许的误差范围
进入函数后开始控制无人机飞向目标点，返回值为bool型，表示是否到达目标点
*************************************************************************/
int range_min = 0;	 // 激光雷达探测范围 最小角度
int range_max = 359; // 激光雷达探测范围 最大角度
void lidar_cb(const sensor_msgs::LaserScan::ConstPtr &scan)
{
	sensor_msgs::LaserScan Laser_tmp;
	Laser_tmp = *scan;
	Laser = *scan;
	int count; // count = 359
	count = Laser.ranges.size();

	// 剔除inf的情况
	for (int i = 0; i < count; i++)
	{
		// 判断是否为inf
		int a = isinf(Laser_tmp.ranges[i]);
		// 如果为inf，则赋值上一角度的值
		if (a == 1)
		{
			Laser_tmp.ranges[i] = 6.0;
		}
	}
	Laser_std = Laser_tmp;
	for (int i = 0; i < count; i++)
	{
		if (i + 180 > 359)
			Laser.ranges[i] = Laser_tmp.ranges[i - 180];
		else
			Laser.ranges[i] = Laser_tmp.ranges[i + 180];
		// cout<<"tmp: "<<i<<" l:"<<Laser_tmp.ranges[i]<<"|| Laser: "<<Laser.ranges[i]<<endl;
	}
	// cout<<"//////////////"<<endl;
	// 计算前后左右四向最小距离
	cal_min_distance();
	cal_best_angle();
}
/************************************************************************
函数 6:避障
控制无人机避障飞向（x, y, z）位置，yaw为目标航向角，error_max为允许的误差范围
进入函数后开始控制无人机飞向目标点，返回值为bool型，表示是否到达目标点
存在的问题：在穿门时容易撞到门框，后续还应调整相关系数
*************************************************************************/
float collision_avoidance_last_position_x = 0;
float collision_avoidance_last_position_y = 0;
float st_vel = 0.5f;
float vel_track[2];
float vel_collision[2];
float vel_sp_body[2];
bool collision_avoidance_flag = false;
bool collision_avoidance(float x, float y, float z, float err_max, double max_vel)
{

	if (distance_c >= 3)
	{
		collision_avoidance_flag = false;
	}
	else
	{
		collision_avoidance_flag = true;
	}
	vel_track[0] = 0;
	vel_track[1] = 0;
	// 3. 计算追踪速度
	float total_distance = sqrt((x - local_pos.pose.pose.position.x) * (x - local_pos.pose.pose.position.x) + (y - local_pos.pose.pose.position.y) * (y - local_pos.pose.pose.position.y));
	if (total_distance > 0.05f)
	{ // 距离大于5cm，才计算追踪速度
		vel_track[0] = st_vel * (x - local_pos.pose.pose.position.x) / total_distance;
		vel_track[1] = st_vel * (y - local_pos.pose.pose.position.y) / total_distance;
	}

	if (isobs(x, y, total_distance))
	{
		mission_pos_cruise(x, y, z, 0, err_max);
	}
	else
	{
		// 速度限幅
		for (int i = 0; i < 2; i++)
		{
			vel_track[i] = satfunc(vel_track[i], max_vel);
		}
		vel_collision[0] = 0;
		vel_collision[1] = 0;

		// 4. 避障策略
		if (collision_avoidance_flag == true)
		{
			distance_cx = distance_c * cos(angle_c / 180 * 3.1415926);
			distance_cy = distance_c * sin(angle_c / 180 * 3.1415926);

			float F_c;

			F_c = 0;

			// 小幅度抑制移动速度
			if (distance_c > 1.5 && distance_c <= 3)
			{
				F_c = 1 * (R_outside - distance_c);
			}

			// 大幅度抑制移动速度
			else if (distance_c <= 1.5)
			{
				F_c = 1 * (R_outside - R_inside) + 3 * (R_inside - distance_c);
			}

			vel_collision[0] = vel_collision[0] - F_c * distance_cx / distance_c;
			vel_collision[1] = vel_collision[1] - F_c * 1.5 * distance_cy / distance_c;
			// 避障速度限幅
			for (int i = 0; i < 2; i++)
			{
				vel_collision[i] = satfunc(vel_collision[i], max_vel);
			}
		}

		vel_sp_body[0] = 0;
		vel_sp_body[1] = 0;
		vel_sp_body[0] = vel_track[0] + vel_collision[0] + 1.5 * cos(best_angle / 180 * 3.1415926);
		vel_sp_body[1] = vel_track[1] + vel_collision[1] * 1.5 + 2.5 * sin(best_angle / 180 * 3.1415926); // dyx

		// 找当前位置到目标点的xy差值，如果出现其中一个差值小，另一个差值大，
		// 且过了一会还是保持这个差值就开始从差值入手。
		// 比如，y方向接近0，但x还差很多，但x方向有障碍，这个时候按discx cy的大小，缓解y的难题。

		float total_vel = sqrt(vel_sp_body[0] * vel_sp_body[0] + vel_sp_body[1] * vel_sp_body[1]);
		if (total_vel > max_vel)
		{
			vel_sp_body[0] = max_vel * vel_sp_body[0] / total_vel;
			vel_sp_body[1] = max_vel * vel_sp_body[1] / total_vel;
		}

		setpoint_raw.type_mask = 1 + 2 + 4 /*+ 8 + 16 + 32*/ + 64 + 128 + 256 + 512 /*+ 1024 + 2048*/;
		setpoint_raw.coordinate_frame = 1;
		setpoint_raw.position.x = local_pos.pose.pose.position.x;
		setpoint_raw.position.y = local_pos.pose.pose.position.y;
		setpoint_raw.position.z = local_pos.pose.pose.position.z;

		// 速度字段（原有逻辑保留，已补全）
		setpoint_raw.velocity.x = vel_sp_body[0];
		setpoint_raw.velocity.y = vel_sp_body[1];
		setpoint_raw.velocity.z = 0.0f; // 保持高度

		// 必选：补全加速度字段（飞控要求，设为0即可）
		setpoint_raw.acceleration_or_force.x = 0.0f;
		setpoint_raw.acceleration_or_force.y = 0.0f;
		setpoint_raw.acceleration_or_force.z = 0.0f;

		// 必选：补全姿态字段（飞控要求，用当前yaw）
		setpoint_raw.yaw = yaw;		  // 偏航角（rad，需从飞控订阅获取）
		setpoint_raw.yaw_rate = 0.0f; // 偏航角速度（保持当前朝向）
	}

	// 11. 日志输出（原有逻辑保留，优化格式）
	ROS_INFO("当前位置(%.2f,%.2f,%.2f) | 目标位置(%.2f,%.2f,%.2f) | 速度(%.2f,%.2f) | 障碍物距离(%.2f) | 追踪速度(%.2f,%.2f) | 避障速度(%.2f,%.2f) | 最佳角度(%.2f) | 最佳角(%.2f,%.2f)",
			 local_pos.pose.pose.position.x,
			 local_pos.pose.pose.position.y,
			 local_pos.pose.pose.position.z,
			 x,
			 y,
			 z,
			 vel_sp_body[0],
			 vel_sp_body[1],
			 distance_c,
			 vel_track[0],
			 vel_track[1],
			 vel_collision[0],
			 vel_collision[1],
			 best_angle,
			 0.8 * cos(best_angle / 180 * 3.1415926),
			 0.8 * sin(best_angle / 180 * 3.1415926));
	if (total_distance <= err_max)
	{
		ROS_INFO("避障完成！");
		collision_avoidance_flag = false;
		return true;
	}
	return false;
}

/************************************************************************
函数 7:饱和函数
限制速度的最大值
*************************************************************************/
float satfunc(float data, float Max)
{
	if (abs(data) > Max)
		return (data > 0) ? Max : -Max;
	else
		return data;
}
/************************************************************************
函数 8:计算最小距离
找到最小距离并计算最小角度
*************************************************************************/
void cal_min_distance()
{
	distance_c = Laser.ranges[0];
	angle_c = 0;
	for (int i = 0; i <= 359; i++)
	{
		if (i > 90 && i < 270)
			continue;
		if (Laser.ranges[i] < distance_c)
		{
			distance_c = Laser.ranges[i];
			angle_c = i;
		}
	}
}
/************************************************************************
函数 9:计算best angle
找到计算best角度
*************************************************************************/
void best(double *front, int &max_start, int &max_end, double round)
{
	int start = 0, end = 0, counting = 0; // 初始化所有变量，max改为max_len避免歧义
	double counts = 0.0, max_len = 0.0;
	for (int i = 0; i < 180; i++)
	{
		if (counting)
		{
			if (front[i] > round)
				counts += front[i];
			else
			{
				end = i - 1; // 修正：结束角是上一个满足条件的角度（i是无效角）
				counting = 0;
				if (max_len < counts)
				{
					max_len = counts;
					max_start = start;
					max_end = end;
				}
				counts = 0;
			}
		}
		else
		{
			if (front[i] > round)
			{
				start = i;
				counts += front[i];
				counting = 1;
			}
			else
			{
				continue;
			}
		}
	}
	// 处理循环结束后仍在统计的尾部区间
	if (counting)
	{
		end = 179; // 尾部区间的结束角是最后一个角度（179）
		if (max_len < counts)
		{
			max_len = counts;
			max_start = start;
			max_end = end;
		}
	}
	// 特殊情况：无任何有效区间，设置为-1表示无效（便于调用者判断）
	if (max_len < 1)
	{
		max_start = -1;
		max_end = -1;
	}
}
/************************************************************************
函数 10:计算加权中位数
通过给定区间计算距离为权重的角度的加权中位数
*************************************************************************/
int cal_middle(double *front, int max_start, int max_end)
{
	double sum = 0.0, temp = 0.0;
	for (int i = max_start; i <= max_end; i++)
		sum += front[i];
	for (int i = max_start; i <= max_end; i++)
	{
		temp += front[i];
		if (temp >= sum / 2)
			return i;
	}
	return (max_start + max_end) / 2;
}
/************************************************************************
函数 11:计算最佳角度
扫描前方180度区间，找到符合条件（宽敞）的角度区间
*************************************************************************/
void cal_best_angle()
{
	double front[180];
	for (int i = 0; i < 180; i++)
	{
		if (i < 90)
			front[i] = Laser.ranges[270 + i];
		else
			front[i] = Laser.ranges[i - 90];
	}
	int ms3, me3, ms2, me2, ms1, me1;
	best(front, ms3, me3, 3.0);
	best(front, ms2, me2, 2.0);
	best(front, ms1, me1, 1.0);
	if (me3 - ms3 > 30)
	{
		int a = cal_middle(front, ms3, me3) / 2 + (ms3 + me3) / 4;
		best_angle = (a < 90) ? 270 + a : a - 90;
	}
	else if (me2 - ms2 > 45)
	{
		int a = cal_middle(front, ms2, me2) / 2 + (ms2 + me2) / 4;
		best_angle = (a < 90) ? 270 + a : a - 90;
	}
	else if (me1 - ms1 > 60)
	{
		int a = cal_middle(front, ms1, me1) / 2 + (ms1 + me1) / 4;
		best_angle = (a < 90) ? 270 + a : a - 90;
	}
	else
	{
		best_angle = 180;
	}
}
/************************************************************************
函数 12:判断有无障碍物函数
计算无人机前方40度范围内是否存在障碍物
*************************************************************************/
int isobs(double x, double y, double total)
{
	double dx = x - local_pos.pose.pose.position.x;
	double dy = y - local_pos.pose.pose.position.y;
	double angle = atan2(dy, dx);
	int str_angle = angle / 3.1415926 * 180 + 180;
	for (int i = str_angle - 20; i < str_angle + 20; i++)
	{
		if (Laser_std.ranges[i] < total)
			return false;
	}
	return true;
}
/************************************************************************
函数 13:判断是否存在圆环
以当前位置开始扫描，基于雷达返回的消息进行分析
*************************************************************************/
bool is_exist_ring()
{
	all_ring.stick_angle.clear(); // 清空历史杆状障碍物
	all_ring.ring_ifo.clear();
	all_ring.FindStick();
	if (all_ring.IsRing())
	{
		all_ring.NearestRing();
		return true;
	}
	else
		return false;
}
/************************************************************************
函数 14:穿环
控制无人机经历
1.寻找圆环
2.对准圆环
3.二次矫正
4.穿越圆环
5.寻找圆环
......
如此反复的过程，直到前方不存在圆环时直接前往终点
进入函数后开始控制无人机寻找圆环进入状态循环，返回值为bool型，表示是否到达目标点
存在的问题：在穿环时位置调整不够精确会撞到环/被卡住，但要求精确后导致飞行时间大大增加
*************************************************************************/
bool cross_ring(double x, double y, double err_max)
{
	double total_distance = sqrt((x - local_pos.pose.pose.position.x) * (x - local_pos.pose.pose.position.x) + (y - local_pos.pose.pose.position.y) * (y - local_pos.pose.pose.position.y));
	if (!find_ring)
	{
		if (!is_exist_ring())
		{
			if (isobs(x, y, total_distance))
				mission_pos_cruise(x, y, ALTITUDE, 0, err_max);
			else
				mission_pos_cruise(local_pos.pose.pose.position.x + 0.5, local_pos.pose.pose.position.y, ALTITUDE, 0, err_max);
		}
		else
		{
			ROS_WARN("发现圆环，环中心位置(%f,%f)", nearest_ring[0] + local_pos.pose.pose.position.x, nearest_ring[1] + local_pos.pose.pose.position.y);
			find_ring = true;
			cross_ring_flag = true;
		}
	}
	else
	{
		switch (mode)
		{
		case 1:
		{
			if (move_in_drone_coordinate(nearest_ring[0] - 1.5, nearest_ring[1], err_max))
			{
				isinit = false;
				ROS_INFO("对准环中心位置，准备穿越！");
				mode = 2;
				is_exist_ring();
			}
			break;
		}
		case 2:
		{
			if (move_in_drone_coordinate(nearest_ring[0] - 0.8, nearest_ring[1], err_max))
			{
				isinit = false;
				ROS_INFO("位置矫正完成！");
				mode = 3;
			}
			break;
		}
		case 3:
		{
			if (collision_in_drone_coordinate(2.5, 0, err_max, 0.2))
			{
				isinit = false;
				ROS_INFO("已穿越圆环！");
				mode = 4;
			}
			break;
		}
		case 4:
		{
			ROS_INFO("寻找下一圆环");
			mode = 1;
			find_ring = false;
			break;
		}
		}
	}
	if (total_distance <= err_max)
	{
		ROS_INFO("穿环完成！");
		cross_ring_flag = false;
		return true;
	}
	return false;
}

/************************************************************************
函数 15:在无人机坐标系下定点飞行
基于当前无人机位置与目标位置进行计算后调用定点飞行函数
*************************************************************************/
bool move_in_drone_coordinate(double x, double y, double err_max)
{
	if (!isinit)
		init_location(init_x, init_y);
	return mission_pos_cruise(x + init_x, y + init_y, 1.32, 0, err_max);
}

/************************************************************************
函数 16:在无人机坐标系下避障
基于当前无人机位置与目标位置进行计算后调用避障函数
*************************************************************************/
bool collision_in_drone_coordinate(double x, double y, double err_max, double max_vel)
{
	if (!isinit)
		init_location(init_x, init_y);
	return collision_avoidance(x + init_x, y + init_y, 1.32, err_max, 0.2);
}

/************************************************************************
函数 17:初始化位置坐标
判断是否需要进行初始化并在必要时进行初始化，防止二次初始化
*************************************************************************/
void init_location(double &init_x, double &init_y)
{
	init_x = local_pos.pose.pose.position.x;
	init_y = local_pos.pose.pose.position.y;
	isinit = true;
}
/************************************************************************
函数 18:计算两点距离
基于Law of Cosines（余弦定理）由无人机雷达得到两边与角度差进行计算
用于后面计算到圆环直径
如有疑问请问团队dhz
*************************************************************************/
double call_len(double angle1, double angle2) // angle1为start，angle2为end
{
	// 利用余弦定理
	double edge1 = Laser_std.ranges[(int)angle1];
	double edge2 = Laser_std.ranges[(int)angle2];
	double dif_angle = angle2 - angle1;
	return sqrt(edge1 * edge1 + edge2 * edge2 - 2 * edge1 * edge2 * cos(dif_angle / 180 * 3.1415926));
}
/************************************************************************
函数 19:计算中位线的长度
基于Law of Cosines（余弦定理）由无人机雷达得到两边与角度差进行计算
用于后面计算到圆环中心的距离
如有疑问请问团队dhz
*************************************************************************/
double call_mid_len(double angle1, double angle2) // angle1为start，angle2为end
{
	// 利用余弦定理
	double edge1 = Laser_std.ranges[(int)angle1];
	double edge2 = Laser_std.ranges[(int)angle2];
	double dif_angle = angle2 - angle1;
	return sqrt(edge1 * edge1 + edge2 * edge2 + 2 * edge1 * edge2 * cos(dif_angle / 180 * 3.1415926)) / 2; // 注意此处变为加号,计算的是已知角度的补角
}
/************************************************************************
函数 20 和 21:两个用于角度化弧度的函数
如有疑问请问团队dhz
*************************************************************************/
double cal_y(double angle)
{
	return -cos(angle / 180 * 3.1415926) * Laser_std.ranges[(int)angle + 90];
}
double cal_x(double angle)
{
	return sin(angle / 180 * 3.1415926) * Laser_std.ranges[(int)angle + 90];
}

/************************************************************************
函数 22:图像回调函数
如有疑问请问团队s0
*************************************************************************/
// 新的用于识别的变量
// 新的识别
//  图像回调相关
cv::Mat current_frame;
bool got_image = false;
string content;
// 二维码检测结果
cv::Point2f qr_center; // 图像中的中心点（像素）
bool qr_detected = false;
// 相机内参（根据相机参数来设置的，后面用把像素点位置换算为三维的坐标）
cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) << 526.0, 0.0, 320.0,
						 0.0, 526.0, 240.0,
						 0.0, 0.0, 1.0);

// 用于识别的函数

// 图像回调函数
void image_cb(const sensor_msgs::ImageConstPtr &msg)
{
	cv_bridge::CvImagePtr cv_ptr;
	try
	{
		cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
	}
	catch (cv_bridge::Exception &e)
	{
		// 处理异常错误，cv_bridge是专门的描述错误的
		ROS_ERROR("cv_bridge exception:%s", e.what());
		return;
	}

	std::vector <cv::Point2f > corners;
	current_frame = cv_ptr->image;

	cv::QRCodeDetector qrDecoder; // 用来找二维码的
	bool detected = qrDecoder.detect(current_frame, corners);
	if(detected){ROS_INFO("QR detected!");}
	content = qrDecoder.detectAndDecode(current_frame);
	if (detected && corners.size() >= 4)
	{
		// 计算四个角点的中心
		cv::Point2f center(0, 0);
		for (int i = 0; i < 4; i++)
		{
			center.x += corners[i].x;
			center.y += corners[i].y;
		}
		center.x = center.x / 4;
		center.y = center.y / 4;
		qr_center = center;
		qr_detected = true;
		ROS_INFO("find QR center in camera: (%.1f, %.1f)", center.x, center.y);
	}
}
/************************************************************************
函数 23:转换为世界坐标系
如有疑问请问团队s0
*************************************************************************/
// 转化为世界坐标系
geometry_msgs::Point change_to_world(float u, float v)
{
	// 1. 像素坐标 → 相机坐标（z=1平面）
	cv::Mat pixel = (cv::Mat_<double>(3, 1) << u, v, 1.0); // 构造一个向量

	cv::Mat cam_point = camera_matrix.inv() * pixel;
	//.inv()是求逆，然后矩阵乘法
	// 根据公式算出相对于相机中心的x,y,z偏移量的向量,相当于一个比例，z=1时的x,y值，再用真正的z*这个向量

	// 2. 用无人机高度计算相机到地面的距离
	double camera_height = local_pos.pose.pose.position.z - 0.1; // 无人机高度 - 相机安装高度

	double x_c = cam_point.at<double>(0) * camera_height;
	double y_c = cam_point.at<double>(1) * camera_height;
	double z_c = camera_height;
	double cos_yaw = cos(yaw);
	double sin_yaw = sin(yaw);

	double x_w = local_pos.pose.pose.position.x + (x_c * cos_yaw - y_c * sin_yaw);
	double y_w = local_pos.pose.pose.position.y + (x_c * sin_yaw + y_c * cos_yaw);
	double z_w = 0.0;
	geometry_msgs::Point p;
	p.x = x_w;
	p.y = y_w;
	p.z = z_w;
	return p;
}
/************************************************************************
函数 24:飞行并搜索二维码
如有疑问请问团队s0
存在的问题：迫于时间原因，识别数字与字母部分还未完成。
*************************************************************************/
void fly(float v)
{
	setpoint_raw.type_mask = 1 + 2 + 4 /*+ 8 + 16 + 32*/ + 64 + 128 + 256 + 512 /*+ 1024 + 2048*/;
	setpoint_raw.coordinate_frame = 1;
	setpoint_raw.position.x = local_pos.pose.pose.position.x;
	setpoint_raw.position.y = local_pos.pose.pose.position.y;
	setpoint_raw.position.z = local_pos.pose.pose.position.z;

	// 速度字段（原有逻辑保留，已补全）
	setpoint_raw.velocity.x = 0.0f;
	setpoint_raw.velocity.y = v;
	setpoint_raw.velocity.z = 0.0f; // 保持高度

	// 必选：补全加速度字段（飞控要求，设为0即可）
	setpoint_raw.acceleration_or_force.x = 0.0f;
	setpoint_raw.acceleration_or_force.y = 0.0f;
	setpoint_raw.acceleration_or_force.z = 0.0f;

	// 必选：补全姿态字段（飞控要求，用当前yaw）
	setpoint_raw.yaw = yaw;		  // 偏航角（rad，需从飞控订阅获取）
	setpoint_raw.yaw_rate = 0.0f; // 偏航角速度（保持当前朝向）
}



