#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <memory>
#include <functional>
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2/convert.h"
#include <string>
#include "serial/serial.h"
#include <tf2/impl/convert.h>
#include "hoist_msgs/msg/gimble.hpp"
using namespace std::chrono_literals;
class hoist_base : public rclcpp::Node {
public:
    hoist_base() : Node("hoist_base"){
        port = this->declare_parameter<std::string>("port", "/dev/ttyUSB0");
        imu_tf_child_frame_id = this->declare_parameter<std::string>("imu_tf_child_frame_id", "imu_link");
        imu_tf_frame_id = this->declare_parameter<std::string>("imu_tf_frame_id", "base_link");
        imu_frame_id = this->declare_parameter<std::string>("odomframe_id", "imu_link");
		odom_child_frame_id = this->declare_parameter<std::string>("odom_tf_child_frame_id", "base_footprint");
        odom_frame_id = this->declare_parameter<std::string>("odom_frame_id", "odom");
        time_offset_in_seconds = this->declare_parameter<double>("time_offset_in_seconds", 0.0);
        broadcast_tf = this->declare_parameter<bool>("broadcast_tf", true);
        imu_linear_acceleration_variance = this->declare_parameter<double>("imu_linear_acceleration_variance", 0.00117);
        imu_angular_velocity_variance = this->declare_parameter<double>("imu_angular_velocity_variance", 0.0000238);
        imu_orientation_variance = this->declare_parameter<double>("imu_orientation_variance", 0.002);
		odom_linear_velocity_variance = this->declare_parameter<double>("odom_linear_velocity_variance", 0.5);
        odom_angular_velocity_variance = this->declare_parameter<double>("odom_angular_velocity_variance", 20);
		timeout = this->declare_parameter<double>("timeout",0.02);
        imu_pub = this->create_publisher<sensor_msgs::msg::Imu>("imu", 100);
		odom_pub = this->create_publisher<nav_msgs::msg::Odometry>("odom", 100);
        subscription_cmd = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 100, std::bind(&hoist_base::cmd_vel_callback, this, std::placeholders::_1));
		subscription_gimble = this->create_subscription<hoist_msgs::msg::Gimble>("gimble", 10, std::bind(&hoist_base::gimble_callback, this, std::placeholders::_1));
        timer_ = this->create_wall_timer(
      5ms, std::bind(&hoist_base::timer_callback, this));
        imu_tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
		odom_tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    }
	bool car_twist_publisher(){
		rclcpp::Duration duration = current_time - last_time;
		double dt = duration.seconds();
		tf2::Matrix3x3 obs_mat;
		obs_mat.setEulerYPR(-yaw.d, pitch.d, roll.d);
		tf2::Quaternion orientation;
		obs_mat.getRotation(orientation);
        double delta_x = (odom_v_x.d * cos(odom_th) - odom_v_y.d * sin(odom_th)) * dt;
        double delta_y = (odom_v_x.d * sin(odom_th) + odom_v_y.d * cos(odom_th)) * dt;
        double delta_th = odom_v_th.d * dt;
		// RCLCPP_INFO(this->get_logger(),"%f",dt);
        odom_x += delta_x;
        odom_y += delta_y;
        odom_th += delta_th;

        //since all odometry is 6DOF we'll need a quaternion created from yaw
		tf2::Quaternion q;
		q.setRPY(0, 0, odom_th);
        geometry_msgs::msg::Quaternion odom_quat = tf2::toMsg(q);
		if (broadcast_tf){
        //first, we'll publish the transform over tf
        geometry_msgs::msg::TransformStamped odom_trans;
        odom_trans.header.stamp = current_time;
        odom_trans.header.frame_id = odom_frame_id.c_str();
        odom_trans.child_frame_id = odom_child_frame_id.c_str();
    
        odom_trans.transform.translation.x = odom_x;
        odom_trans.transform.translation.y = odom_y;
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation = odom_quat;
 
        //send the transform
        odom_tf_broadcaster->sendTransform(odom_trans);
		}
        //next, we'll publish the odometry message over ROS
        nav_msgs::msg::Odometry odom;
        odom.header.stamp = current_time;
        odom.header.frame_id = odom_frame_id.c_str();
        odom.child_frame_id = odom_child_frame_id.c_str();
    
        //set the position
        odom.pose.pose.position.x = odom_x;
        odom.pose.pose.position.y = odom_y;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = odom_quat;
   
        //set the velocity
        odom.twist.twist.linear.x = odom_v_x.d;
        odom.twist.twist.linear.y = odom_v_y.d;
        odom.twist.twist.angular.z = odom_v_th.d;
		odom.twist.covariance[0] = odom_linear_velocity_variance;
		odom.twist.covariance[7] = 30;
		odom.twist.covariance[35] = odom_angular_velocity_variance;
        //publish the message
        odom_pub->publish(odom);
        last_time = current_time;
	}
	bool imu_publisher(){
		tf2::Matrix3x3 obs_mat;
		obs_mat.setEulerYPR(-yaw.d, pitch.d, roll.d);
		tf2::Quaternion orientation;
		obs_mat.getRotation(orientation);
		if (!zero_orientation_set)
		{
			zero_orientation = orientation;
			zero_orientation_set = true;
		}

		//http://answers.ros.org/question/10124/relative-rotation-between-two-quaternions/
		tf2::Quaternion differential_rotation;
		differential_rotation = zero_orientation.inverse() * orientation;

		rclcpp::Time measurement_time = this->get_clock()->now() + rclcpp::Duration::from_seconds(time_offset_in_seconds);

		// publish imu message
		imu.header.stamp = measurement_time;
		imu.header.frame_id = imu_frame_id.c_str();
		// quaternionTFToMsg(differential_rotation, imu.orientation); // 把tf四元数转化为geomsg四元数

		imu.angular_velocity.x = vel_angular[0].d; // 角速度
		imu.angular_velocity.y = vel_angular[1].d;
		imu.angular_velocity.z = vel_angular[2].d;

		imu.linear_acceleration.x = accel[0].d; // 线加速度
		imu.linear_acceleration.y = accel[1].d;
		imu.linear_acceleration.z = accel[2].d;

        imu.orientation.x = differential_rotation.x();
        imu.orientation.y = differential_rotation.y();
        imu.orientation.z = differential_rotation.z();
        imu.orientation.w = differential_rotation.w();

		imu.linear_acceleration_covariance[0] = imu_linear_acceleration_variance; // 线性加速度协方差 = 线性加速度标准差?
		imu.linear_acceleration_covariance[4] = imu_linear_acceleration_variance;
		imu.linear_acceleration_covariance[8] = imu_linear_acceleration_variance;

		imu.angular_velocity_covariance[0] = imu_angular_velocity_variance;
		imu.angular_velocity_covariance[4] = imu_angular_velocity_variance;
		imu.angular_velocity_covariance[8] = imu_angular_velocity_variance;

		imu.orientation_covariance[0] = imu_orientation_variance;
		imu.orientation_covariance[4] = imu_orientation_variance;
		imu.orientation_covariance[8] = imu_orientation_variance;
		imu_pub->publish(imu);
        geometry_msgs::msg::TransformStamped t;

                                // Read message content and assign it to
                                // corresponding tf variables
                                
								// publish temperature message
								// publish tf transform
		if (broadcast_tf)
		{
            t.header.stamp = this->get_clock()->now();
            t.header.frame_id = imu_tf_frame_id.c_str();
            t.child_frame_id = imu_tf_child_frame_id.c_str();
            t.transform.rotation.x = differential_rotation.x();
            t.transform.rotation.y = differential_rotation.y();
            t.transform.rotation.z = differential_rotation.z();
            t.transform.rotation.w = differential_rotation.w();
			imu_tf_broadcaster->sendTransform(t);
		}
	}
private:
	rclcpp::Time current_time = this->get_clock()->now();
	rclcpp::Time last_time = this->get_clock()->now();
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    serial::Serial ser;
	tf2::Quaternion orientation;
	tf2::Quaternion zero_orientation;
    std::string input;
	std::string read;
    double imu_linear_acceleration_variance;
    double imu_angular_velocity_variance;
    double imu_orientation_variance;
	double odom_linear_velocity_variance;
    double odom_angular_velocity_variance;
    size_t count_;
    float ratio = 1000.0f ;   //转速转换比例，执行速度调整比例
    float D = 0.2680859f ;    //两轮间距，单位是mm
    char const_num=2;
    float pi=3.14159f;
    const int g = 9.8;
    float const_=180.0f;
    int data_packet_start;
    float linear_temp=0,angular_temp=0;//转化后得到的线速度和角速度
    sensor_msgs::msg::Imu imu;
    unsigned char data_terminal0='s';	//“s"字符
    unsigned char data_terminal1='e';	//“e"字符
    unsigned long baud = 115200;
    unsigned char speed_data[16]={0};
    std::string port;
    std::string imu_tf_child_frame_id;
    std::string imu_tf_frame_id;
    std::string imu_frame_id;
	std::string odom_frame_id;
	std::string odom_child_frame_id;
    bool broadcast_tf;
    tf2::Transform transform;
    double time_offset_in_seconds;
	double odom_x=0;
	double odom_y=0;
	double odom_th=0;
	rclcpp::Duration d = rclcpp::Duration::from_seconds(1.0);
    union float16Data	//union的作用为实现char数组和float之间的转换
	{
	float d;
	unsigned char data[4];
	}accel[3],vel_angular[3],yaw,roll,pitch,cmd_x,cmd_y,cmd_z,odom_v_x,odom_v_y,odom_v_th;
    void timer_callback(){
		current_time = now();
        try
		{
			if (ser.isOpen()) // 端口打开
			{
				// read string from serial device 从串行设备读取字符串
				if (ser.available())
				{
					read = ser.read(ser.available());
					input += read;

					// 读到的字符串长度>=33  输入中可能有完整的包
					while (input.length() >= 43) // while there might be a complete package in input
					{
						//parse for data packets 解析数据包
						data_packet_start = input.find("\x55\x51"); // 找开头的标记位
						// int data_packet_twice = input.find("\x55\x52")-data_packet_start;
						if (data_packet_start != std::string::npos)
						{	
							// ROS_INFO("%d after if",data_packet_start);
							if ((input.length() >= data_packet_start + 43) && (input.compare(data_packet_start + 11, 2, "\x55\x52") == 0) && (input.compare(data_packet_start + 22, 2, "\x55\x53") == 0)) //check if positions 26,27 exist, then test values
							{
								for(int i = 0;i<3;i++){
									vel_angular[0].data[1+i] = input[data_packet_start+2+i];
									vel_angular[1].data[1+i] = input[data_packet_start+5+i];
									vel_angular[2].data[1+i] = input[data_packet_start+8+i];
									accel[0].data[1+i] = input[data_packet_start+13+i];
									accel[1].data[1+i] = input[data_packet_start+16+i];
									accel[2].data[1+i] = input[data_packet_start+19+i];
									yaw.data[1+i] = input[data_packet_start+24+i];
									pitch.data[1+i] = input[data_packet_start+27+i];
									roll.data[1+i] = input[data_packet_start+30+i];
									odom_v_x.data[1+i] = input[data_packet_start+33+i];
									odom_v_y.data[1+i] = input[data_packet_start+36+i];
									odom_v_th.data[1+i] = input[data_packet_start+39+i];
									// yaw.data[1+i] = 0;
									// roll.data[1+i] = 0;
									// pitch.data[1+i] = 0;
								}
								hoist_finish = input[data_packet_start+42];
								imu_publisher();
								car_twist_publisher();
								// RCLCPP_INFO_STREAM(this->get_logger(), "My log message " << yaw.d * pi<<roll.d * pi<<pitch.d * pi);
								input.erase(0, data_packet_start + 33); // delete everything up to and including the processed packet
							}
							else
							{	
								
								if (input.length() >= data_packet_start + 33)
								{
									input.erase(0, data_packet_start + 1); // delete up to false data_packet_start character so it is not found again
								}
								else
								{
									// do not delete start character, maybe complete package has not arrived yet
									input.erase(0, data_packet_start);
								}
							}
						}
						else
						{
							// no start character found in input, so delete everything
							// ROS_ERROR("NO START CHARACTER");
							input.clear();
						}
					}
				}
			}
			else
			{
				// try and open the serial port
				try
				{
					ser.setPort(port);
					ser.setBaudrate(115200);
					serial::Timeout to = serial::Timeout::simpleTimeout(1000);
					ser.setTimeout(to);
					ser.open();
				}
				catch (serial::IOException &e)
				{
					RCLCPP_ERROR_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), d.nanoseconds()/1000000,"Unable to open serial port " << ser.getPort() << ". Trying again in 5 seconds.");
					rclcpp::sleep_for(100ms);
				}

				if (ser.isOpen())
				{
					// ROS_DEBUG_STREAM("Serial port " << ser.getPort() << " initialized and opened.");
				}
			};

			if (ser.isOpen())
			{
				// ROS_DEBUG_STREAM("Serial port " << ser.getPort() << " initialized and opened.");
			}

		
		}
    catch (serial::IOException &e)
		{
			RCLCPP_ERROR_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), d.nanoseconds()/1000000,"Error reading from the serial port " << ser.getPort() << ". Closing connection.");
			ser.close();
		}
		if ((current_time - cmd_vel_last_received).nanoseconds()/1e9 > timeout){
			for(int i=0;i<4;i++)	//将左右轮速度存入数组中发送给串口
			{
				speed_data[i+1]=cmd_x.data[i];
				speed_data[i+5]=cmd_y.data[i];
				speed_data[i+9]=cmd_z.data[i];
			}
			size_t bytes_written = ser.write(speed_data,16);
			cmd_vel_last_received = this->get_clock()->now();
		}
    }

    void cmd_vel_callback(const geometry_msgs::msg::Twist msg) 
	{
        unsigned long baud = 115200;	//小车串口波特率
		// ser.open();
		// linear_temp = ratio*limit_vel_speed ;
		cmd_x.d = msg.linear.x;
		cmd_y.d = 1.7*msg.linear.y;
		cmd_z.d = msg.angular.z;
		//存入数据到要发布的左右轮速度消息

		for(int i=0;i<4;i++)	//将左右轮速度存入数组中发送给串口
		{
			speed_data[i+1]=cmd_x.data[i];
			speed_data[i+5]=cmd_y.data[i];
			speed_data[i+9]=cmd_z.data[i];
		}

		//在写入串口的左右轮速度数据后加入”/r/n“
		speed_data[0]=data_terminal0;
		speed_data[13]=0;
		speed_data[14]=0;
		speed_data[15]=data_terminal1;
		//写入数据到串口
		size_t bytes_written = ser.write(speed_data,16);
		cmd_vel_last_received = this->get_clock()->now();

    }
	void gimble_callback(const hoist_msgs::msg::Gimble msg) 
	{
        unsigned long baud = 115200;	//小车串口波特率
		ser.setPort(port.c_str());
		ser.setBaudrate(115200);
		serial::Timeout to = serial::Timeout::simpleTimeout(1000);
		ser.setTimeout(to);
		// ser.open();
		// linear_temp = ratio*limit_vel_speed ;
		//存入数据到要发布的左右轮速度消息

		for(int i=0;i<4;i++)	//将左右轮速度存入数组中发送给串口
		{
			speed_data[i+1]=0;
			speed_data[i+5]=0;
			speed_data[i+9]=0;
		}

		//在写入串口的左右轮速度数据后加入”/r/n“
		speed_data[0]=data_terminal0;
		speed_data[13]=msg.mod_1;
		speed_data[14]=msg.mod_2;
		speed_data[15]=data_terminal1;
		//写入数据到串口
		size_t bytes_written = ser.write(speed_data,16);
    }

    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub;
	rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_cmd;
	rclcpp::Subscription<hoist_msgs::msg::Gimble>::SharedPtr subscription_gimble;
    std::unique_ptr<tf2_ros::TransformBroadcaster> imu_tf_broadcaster;
	std::unique_ptr<tf2_ros::TransformBroadcaster> odom_tf_broadcaster;
	rclcpp::Time cmd_vel_last_received = this->get_clock()->now();
	uint8_t hoist_finish;
    bool zero_orientation_set;
	double timeout;
};


int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<hoist_base>());
    rclcpp::shutdown();
    return 0;
}