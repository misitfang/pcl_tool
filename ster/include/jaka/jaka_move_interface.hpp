#ifndef JAKA_MOVE_INTERFACE_H
#define JAKA_MOVE_INTERFACE_H

#include "JAKAZuRobot.h"
#include <vector>
#include <memory>

class JakaMoveInterface
{
public:
    typedef std::shared_ptr<JakaMoveInterface> Ptr_;

    /**
     * @brief Construct a new Jaka Move Interface object
     * 
     * @param robot_ip 机械臂ip，默认为169.254.232.101
     */
    JakaMoveInterface(const std::string& robot_ip):robot_ip_(robot_ip){}
    ~JakaMoveInterface(){};

    void init();
    void deinit();

    /**
     * @brief 开启力控传感器，并初始化
     * 
     */
    void enableForceSensor();

    /**
     * @brief 力控恒力模式下拔枪，边拔枪边调整
     * @param[in] ft_damping: 阻尼力大小
     * @param[in] ft_constant: 恒力大小 +往前进, -往后退
     * 
     */
    void admitCtrlExtra(
        const double& ft_damping, const double& ft_constant, 
        const double& rx_move, const double& ry_move,
        const double& max_dep, 
        const double& lower_sum_rx, const double& upper_sum_rx, const double& lower_sum_ry, const double& upper_sum_ry);

    /**
     * @brief 关闭力控传感器
     * 
     */
    void disableForceSensor();

    /**
     * @brief 启动机械臂
     * 
     * @return true 启动完成
     * @return false 启动未完成
     */
    bool startup(); 

    /**
     * @brief 关闭机械臂
     * 
     * @return true 关闭完成
     * @return false 关闭未完成
     */
    bool shutdown();

    /**
     * @brief 设置机械臂负载
     * 
     * @param[in] mass 设置负载重量
     * @param[in] cx 负载重心x坐标
     * @param[in] cy 负载重心y坐标
     * @param[in] cz 负载重心z坐标
     * @return true 完成设置
     * @return false 未完成设置
     */
    bool setPayload(const double& mass, const double& cx, const double& cy, const double& cz);

    /**
     * @brief 设置TCP
     * 
     * @param[in] tool_id 设置的tcp id
     * @param[in] tool_name 设置的tcp名称
     * @param[in] tx 设置tcp的x位移
     * @param[in] ty 设置tcp的y位移
     * @param[in] tz 设置tcp的z位移
     * @return true 完成设置
     * @return false 未完成设置
     */
    bool setTcp(const int& tool_id, const char* tool_name, const double& tx, const double& ty, const double& tz);
    
    /**
     * @brief 获取机械臂的关节坐标及基座坐标系下的tcp坐标
     * 
     * @param[out] joint_pos 获取到的关节坐标
     * @param[out] base_tcp 获取到的基座坐标系下tcp坐标
     */
    void getRobotPosition(std::vector<double>& joint_pos, std::vector<double>& base_tcp);

    /**
     * @brief 机械臂关节运动
     * 
     * @param[in] joint_target 关节坐标值 
     * @param[in] acc 加速度 
     * @param[in] speed 速度
     */
    void jointMove(const std::vector<double>& joint_target, const double& acc, const double& speed);

    /**
     * @brief 输入tcp坐标系下目标点位置，机械臂走直线运动
     * 
     * @param[in] pose_tcp_target tcp坐标系下目标点位置
     * @param[in] acc 加速度
     * @param[in] speed 速度
     * @param[in] is_block 是否阻塞
     * @return true 
     * @return false 
     */
    bool linearMoveBasedTcp(const std::vector<double>& pose_tcp_target, const double& acc, const double& speed, const int& is_block);

    /**
     * @brief 输入tcp坐标系下目标点位置，机械臂经过逆向运动学后走关节运动
     * 
     * @param[in] pose_tcp_target 
     * @param[in] acc 
     * @param[in] speed 
     * @param[in] is_block 
     */
    void jointMoveBasedTcp(const std::vector<double>& pose_tcp_target, const double& acc, const double& speed, const int& is_block);

    /**
     * @brief Get the Origin Tcp Force object
     * 
     * @param origin_tcp_torque 
     * @param actual_tcp_torque 
     */
    void getOriginTcpForce(std::vector<double>& origin_tcp_torque, std::vector<double>& actual_tcp_torque);

    /**
     * @brief 机械臂运动停止
     **/
    void motionAbort();

    /**
     * @brief 设置机械臂碰撞等级，
     * 0 为关闭碰撞，1 为碰撞阈值 25N，
     * 2 为碰撞阈值 50N，3 为碰撞阈值 75N，
     * 4 为碰撞阈值 100N，5 为碰撞阈值 125N，
     **/
    void setCollisionLevel(const int& level);

    /**
     * @brief 查询机械臂运动是否停止
     **/
    bool isInPos();

    bool isInTarget(
        const std::vector<double>& pose_base_tcp1, 
        const std::vector<double>& pose_base_tcp2, 
        const std::vector<double>& pose_tcp_target);

    bool inverseKinematics(const std::vector<double>& pose_base_target, std::vector<double>& joint_pos);

    double depthCalculator(const std::vector<double>& pose_tcp1, const std::vector<double>& pose_tcp2);

private:
    std::string robot_ip_;
    JAKAZuRobot jaka_robot_;
    RobotStatus status_;
};

#endif