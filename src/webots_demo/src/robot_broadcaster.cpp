#include <signal.h>
#include <std_msgs/String.h>
#include "ros/ros.h"

#include <webots_ros/set_float.h>
#include <webots_ros/set_int.h>
#include <webots_ros/Int32Stamped.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <tf/transform_broadcaster.h>
using namespace std;
#define TIME_STEP 32    //时钟
#define NMOTORS 2       //电机数量
#define MAX_SPEED 2.0   //电机最大速度

ros::NodeHandle *n;

static int controllerCount;
static vector<string> controllerList; 
string controllername ;
ros::ServiceClient timeStepClient;          //时钟通讯客户端
webots_ros::set_int timeStepSrv;            //时钟服务数据
double GPSvalues[6];                        // gps数据
int gps_flag=1;
double Inertialvalues[4];                   // inertial_unit数据
double liner_speed=0;
double angular_speed=0;
/*******************************************************
* Function name ：controllerNameCallback
* Description   ：控制器名回调函数，获取当前ROS存在的机器人控制器
* Parameter     ：
        @name   控制器名
* Return        ：无
**********************************************************/
void controllerNameCallback(const std_msgs::String::ConstPtr &name) { 
    controllerCount++; 
    controllerList.push_back(name->data);//将控制器名加入到列表中
controllername = controllerList.back().c_str();
    ROS_INFO("Controller #%d: %s.", controllerCount, controllerList.back().c_str());
}
void broadcastTransform()
{
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(-GPSvalues[2],GPSvalues[0],GPSvalues[1]));// 设置原点
    tf::Quaternion q(Inertialvalues[0],Inertialvalues[1],Inertialvalues[2],Inertialvalues[3]);// 四元数 ->欧拉角
    q = q.inverse();// 反转四元数
    transform.setRotation(q); //设置旋转数据
    br.sendTransform(tf::StampedTransform(transform,ros::Time::now(),"odom","base_link"));// 发送tf坐标关系
    transform.setIdentity();

//tf::Quaternion q2(0.707107,0,-0.707107,0);
/*
tf::Quaternion q2;
transform.setOrigin(tf::Vector3(0,0.3,0.18));
q2.setRPY(0,0,-1.570796);
q2 = q2.inverse();
transform.setRotation(q2);*/
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link", "/load_good_bot/Sick_LMS_291"));
}
void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr &values)
{
    GPSvalues[0] = values->latitude;// 纬度
    GPSvalues[1] = values->altitude;// 海拔高度
    GPSvalues[2] = values->longitude;// 经度
    broadcastTransform(); // tf坐标转换
}
void inertial_unitCallback(const sensor_msgs::Imu::ConstPtr &values)
{
    Inertialvalues[0] = values->orientation.x; 
    Inertialvalues[1] = values->orientation.y;
    Inertialvalues[2] = values->orientation.z;
    Inertialvalues[3] = values->orientation.w;
    broadcastTransform(); // tf坐标转换
}

/*******************************************************
* Function name ：quit
* Description   ：退出函数
* Parameter     ：
        @sig   信号
* Return        ：无
**********************************************************/
void quit(int sig) {
    ROS_INFO("User stopped the '/robot' node.");
    timeStepSrv.request.value = 0; 
    timeStepClient.call(timeStepSrv); 
    ros::shutdown();
    exit(0);
}

int main(int argc, char **argv) {
    setlocale(LC_ALL, ""); // 用于显示中文字符
    string controllerName;
    // 在ROS网络中创建一个名为robot_init的节点
    ros::init(argc, argv, "robot_init", ros::init_options::AnonymousName);
    n = new ros::NodeHandle;
    // 截取退出信号
    signal(SIGINT, quit);

    // 订阅webots中所有可用的model_name
    ros::Subscriber nameSub = n->subscribe("model_name", 100, controllerNameCallback);
    while (controllerCount == 0 || controllerCount < nameSub.getNumPublishers()) {
        ros::spinOnce();
    }
    ros::spinOnce();
    // 服务订阅time_step和webots保持同步
    timeStepClient = n->serviceClient<webots_ros::set_int>("/load_good_bot/robot/time_step");
    timeStepSrv.request.value = TIME_STEP;

    // 如果在webots中有多个控制器的话，需要让用户选择一个控制器
	/*
    if (controllerCount == 1)
        controllerName = controllerList[0];
    else {
        int wantedController = 0;
        cout << "Choose the # of the controller you want to use:\n";
        cin >> wantedController;
        if (1 <= wantedController && wantedController <= controllerCount)
        controllerName = controllerList[wantedController - 1];
        else {
        ROS_ERROR("Invalid number for controller choice.");
        return 1;
        }
    }
    ROS_INFO("Using controller: '%s'", controllerName.c_str());*/
    // 退出主题，因为已经不重要了
    nameSub.shutdown();

    ros::ServiceClient gps_Client;          
    webots_ros::set_int gps_Srv;            
    ros::Subscriber gps_sub;
    gps_Client = n->serviceClient<webots_ros::set_int>("/load_good_bot/gps/enable"); // 使能GPS服务
    gps_Srv.request.value = TIME_STEP;
    // 判断gps使能服务是否成功
    if (gps_Client.call(gps_Srv) && gps_Srv.response.success) {
        ROS_INFO("gps enabled.");
        // 订阅gps话题，获取数据
        gps_sub = n->subscribe("/load_good_bot/gps/values", 1, gpsCallback);
        ROS_INFO("Topic for gps initialized.");
        while (gps_sub.getNumPublishers() == 0) {
        }
        ROS_INFO("Topic for gps connected.");
    } else {
        if (!gps_Srv.response.success)
        ROS_ERROR("Failed to enable gps.");
        return 1;
    }
    ros::ServiceClient inertial_unit_CgetNumPublisherslient;          
    webots_ros::set_int inertial_unit_Srv;            
    ros::Subscriber inertial_unit_sub;
    ros::ServiceClient inertial_unit_Client;  
    inertial_unit_Client = n->serviceClient<webots_ros::set_int>("/load_good_bot/inertial_unit/enable"); //订阅IMU使能服务
    inertial_unit_Srv.request.value = TIME_STEP;
    // 判断是否使能成功
    if (inertial_unit_Client.call(inertial_unit_Srv) && inertial_unit_Srv.response.success) {
        ROS_INFO("inertial_unit enabled.");
        // 获取话题数据
        inertial_unit_sub = n->subscribe("/load_good_bot/inertial_unit/roll_pitch_yaw", 1, inertial_unitCallback);
        ROS_INFO("Topic for inertial_unit initialized.");
        while (inertial_unit_sub.getNumPublishers() == 0) {

        }
        ROS_INFO("Topic for inertial_unit connected.");
    } else {
        if (!inertial_unit_Srv.response.success)
        ROS_ERROR("Failed to enable inertial_unit.");
        return 1;
    }
    ros::ServiceClient lidar_Client;          
    webots_ros::set_int lidar_Srv;            
    lidar_Client = n->serviceClient<webots_ros::set_int>("/load_good_bot/Sick_LMS_291/enable"); // 订阅lidar使能服务
    lidar_Srv.request.value = TIME_STEP;
    // 判断是否使能成功
    if (lidar_Client.call(lidar_Srv) && lidar_Srv.response.success) {
        ROS_INFO("gps enabled.");
    } else {
        if (!lidar_Srv.response.success)
        ROS_ERROR("Failed to enable lidar.");
        return 1;
    }
    // main loop
    while (ros::ok()) {
        if (!timeStepClient.call(timeStepSrv) || !timeStepSrv.response.success) {
        ROS_ERROR("Failed to call service time_step for next step.");
        break;
        }
        ros::spinOnce();
    }
    timeStepSrv.request.value = 0;
    timeStepClient.call(timeStepSrv);
    ros::shutdown(); 
    return 0;
}
