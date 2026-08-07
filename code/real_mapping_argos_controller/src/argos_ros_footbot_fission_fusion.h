#ifndef ARGOS_ROS_FOOTBOT_FISSION_FUSION_H_
#define ARGOS_ROS_FOOTBOT_FISSION_FUSION_H_

#include <argos3/core/simulator/simulator.h>
#include <argos3/core/simulator/space/space.h>
#include <argos3/core/simulator/physics_engine/physics_engine.h>
#include <argos3/core/control_interface/ci_controller.h>
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_proximity_sensor.h>
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_sensor.h>
#include <argos3/plugins/robots/generic/control_interface/ci_simple_radios_actuator.h>
#include <argos3/plugins/robots/generic/control_interface/ci_simple_radios_sensor.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>

#include <string>
#include <sstream>
#include <memory>
#include <vector>
#include <cstring>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

using namespace argos;

/**
 * @class ArgosRosFootbot
 * @brief ARGoS3 controller bridging simulated FootBot sensors/actuators with ROS 2 topics.
 */
class ArgosRosFootbot : public CCI_Controller
{
private:
    /* ROS 2 Publishers */
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr virtualProxPublisher_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr neighborPublisher_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr radioRxPublisher_;

    /* ROS 2 Subscribers */
    rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr tagPoseSubscriber_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr radioTxSubscriber_;

    /* ARGoS Sensor and Actuator Interfaces */
    CCI_FootBotProximitySensor       *m_pcProximity;
    CCI_RangeAndBearingSensor        *m_pcRABS;
    CCI_SimpleRadiosActuator         *m_pcRadioTx;
    CCI_SimpleRadiosSensor           *m_pcRadioRx;

    /* Internal Radio Transmission Buffer State */
    std_msgs::msg::Float64MultiArray pendingRadioMsg_;
    bool hasPendingRadioMsg_;

public:
    ArgosRosFootbot();
    virtual ~ArgosRosFootbot();

    /* ROS 2 Node Instance Handle */
    std::shared_ptr<rclcpp::Node> nodeHandle_;

    /* ARGoS Controller Lifecycle Methods */
    virtual void Init(TConfigurationNode &t_node);
    virtual void ControlStep();
    virtual void Reset();
    virtual void Destroy() {}

    /* ROS 2 Callbacks */
    void tagPoseCallback(const geometry_msgs::msg::Pose2D::SharedPtr pose);
    void radioTxCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
};

#endif