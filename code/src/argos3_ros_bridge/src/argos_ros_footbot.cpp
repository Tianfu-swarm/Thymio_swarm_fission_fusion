#include "argos_ros_footbot.h"
#include <cmath>
#include <cstring>
using namespace std;
using std::placeholders::_1;

ArgosRosFootbot::ArgosRosFootbot() : m_pcWheels(NULL),
                                     m_pcProximity(NULL),
                                     m_pcRABA(NULL),
                                     m_pcRABS(NULL),
                                     m_pcRadioTx(NULL),
                                     m_pcRadioRx(NULL),
                                     leftSpeed(0),
                                     rightSpeed(0),
                                     hasPendingRadioMsg_(false) {}

ArgosRosFootbot::~ArgosRosFootbot() {}

void ArgosRosFootbot::Init(TConfigurationNode &t_node)
{
    int argc = 0;
    char **argv = nullptr;
    if (!rclcpp::ok()) rclcpp::init(argc, argv);

    std::string node_name = GetId() + "_argos3_ros_bridge";
    nodeHandle_ = std::make_shared<rclcpp::Node>(node_name);

    // Automatically assign tag_id based on the robot's name (e.g., "bot7" -> 7)
    int tag_id = 1; 
    std::string bot_name = GetId();
    if (bot_name.find("bot") != std::string::npos) {
        tag_id = std::stoi(bot_name.substr(3)); 
    }

    // ── Topic name builders ──────────────────────────────────────────────────
    stringstream vProxTopic, neighborTopic, cmdVelTopic, tagTopic;
    stringstream radioRxTopic, radioTxTopic;

    vProxTopic   << "/" << GetId() << "/virtual_prox";
    neighborTopic << "/" << GetId() << "/neighbors";
    cmdVelTopic  << "/" << GetId() << "/cmd_vel";
    tagTopic     << "/tag_tracker/pose/tag_" << tag_id;
    radioRxTopic << "/" << GetId() << "/radio_rx";   // ARGoS → ROS  (received)
    radioTxTopic << "/" << GetId() << "/radio_tx";   // ROS → ARGoS  (to send)

    // ── Publishers ───────────────────────────────────────────────────────────
    virtualProxPublisher_ = nodeHandle_->create_publisher<std_msgs::msg::Float64MultiArray>(
        vProxTopic.str(), 10);
    neighborPublisher_ = nodeHandle_->create_publisher<std_msgs::msg::Float64MultiArray>(
        neighborTopic.str(), 10);
    radioRxPublisher_ = nodeHandle_->create_publisher<std_msgs::msg::Float64MultiArray>(
        radioRxTopic.str(), 10);

    // ── Subscribers ──────────────────────────────────────────────────────────
    cmdVelSubscriber_ = nodeHandle_->create_subscription<geometry_msgs::msg::Twist>(
        cmdVelTopic.str(), 10,
        std::bind(&ArgosRosFootbot::cmdVelCallback, this, _1));

    auto best_effort_qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
    tagPoseSubscriber_ = nodeHandle_->create_subscription<geometry_msgs::msg::Pose2D>(
        tagTopic.str(), best_effort_qos,
        std::bind(&ArgosRosFootbot::tagPoseCallback, this, _1));

    radioTxSubscriber_ = nodeHandle_->create_subscription<std_msgs::msg::Float64MultiArray>(
        radioTxTopic.str(), 10,
        std::bind(&ArgosRosFootbot::radioTxCallback, this, _1));

    // ── ARGoS actuators / sensors ────────────────────────────────────────────
    m_pcProximity = GetSensor<CCI_FootBotProximitySensor>("footbot_proximity");
    m_pcWheels    = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
    m_pcRABA      = GetActuator<CCI_RangeAndBearingActuator>("range_and_bearing");
    m_pcRABS      = GetSensor<CCI_RangeAndBearingSensor>("range_and_bearing");
    m_pcRadioTx   = GetActuator<CCI_SimpleRadiosActuator>("simple_radios");
    m_pcRadioRx   = GetSensor<CCI_SimpleRadiosSensor>("simple_radios");

    Reset();
}

void ArgosRosFootbot::ControlStep()
{
    rclcpp::spin_some(nodeHandle_);

    // ── 1. Virtual proximity → ROS ───────────────────────────────────────────
    const CCI_FootBotProximitySensor::TReadings &tProxReads = m_pcProximity->GetReadings();
    std_msgs::msg::Float64MultiArray raw_prox_msg;
    raw_prox_msg.data.reserve(tProxReads.size());
    for (size_t i = 0; i < tProxReads.size(); ++i)
        raw_prox_msg.data.push_back(static_cast<double>(tProxReads[i].Value));
    virtualProxPublisher_->publish(raw_prox_msg);

    // ── 2. Range-and-bearing neighbours → ROS ───────────────────────────────
    const CCI_RangeAndBearingSensor::TReadings &tRabReads = m_pcRABS->GetReadings();
    std_msgs::msg::Float64MultiArray neighbor_msg;
    for (size_t i = 0; i < tRabReads.size(); ++i) {
        neighbor_msg.data.push_back(tRabReads[i].Range);
        neighbor_msg.data.push_back(tRabReads[i].HorizontalBearing.GetValue());
    }
    neighborPublisher_->publish(neighbor_msg);

    // ── 3. Radio TX: send pending message from ROS ───────────────────────────
    // Layout: [round_id, v0, v1, ...] — each double packed as 8 raw bytes.
    if (hasPendingRadioMsg_) {
        CByteArray cData;
        for (double val : pendingRadioMsg_.data) {
            uint64_t raw;
            memcpy(&raw, &val, sizeof(double));
            for (int b = 0; b < 8; ++b)
                cData << static_cast<UInt8>((raw >> (b * 8)) & 0xFF);
        }
        // Interface 0 corresponds to the "wifi" simple_radio medium
        m_pcRadioTx->GetInterfaces()[0].Messages.push_back(cData);
        hasPendingRadioMsg_ = false;
    }

    // ── 4. Radio RX: forward received messages to ROS ────────────────────────
    const CCI_SimpleRadiosSensor::SInterface &rxIface =
        m_pcRadioRx->GetInterfaces()[0];

    for (const CByteArray &raw_msg : rxIface.Messages) {
        std_msgs::msg::Float64MultiArray ros_msg;
        CByteArray copy = raw_msg;
        while (copy.Size() >= 8) {
            uint64_t raw = 0;
            for (int b = 0; b < 8; ++b) {
                UInt8 byte; copy >> byte;
                raw |= (static_cast<uint64_t>(byte) << (b * 8));
            }
            double val;
            memcpy(&val, &raw, sizeof(double));
            ros_msg.data.push_back(val);
        }
        if (!ros_msg.data.empty())
            radioRxPublisher_->publish(ros_msg);
    }

    // NOTE: m_pcWheels->SetLinearVelocity is intentionally NOT called here.
    // The physical robot drives itself; ARGoS position is updated via tagPoseCallback.
}

// ── Callbacks ────────────────────────────────────────────────────────────────

void ArgosRosFootbot::tagPoseCallback(const geometry_msgs::msg::Pose2D::SharedPtr pose)
{
    // 1. Math Safety Net: Ignore corrupted matrices
    if (std::isnan(pose->x) || std::isnan(pose->y) || std::isnan(pose->theta) ||
        std::isinf(pose->x) || std::isinf(pose->y) || std::isinf(pose->theta))
    {
        return;
    }

    // 2. Physical Arena Safety Net: Ignore out-of-bounds coordinates
    // Arena bounds (with a 0.05m safety margin): X = -0.35 to 3.15 | Y = -0.35 to 2.55
    if (pose->x < -0.35 || pose->x > 3.15 || 
        pose->y < -0.35 || pose->y > 2.55) 
    {
        return; // Position is outside the simulator map! Ignore this camera frame.
    }

    argos::CSimulator &cSim = argos::CSimulator::GetInstance();
    try {
        argos::CEntity &cEntity     = cSim.GetSpace().GetEntity(GetId());
        argos::CFootBotEntity &cFB  = dynamic_cast<argos::CFootBotEntity &>(cEntity);

        argos::CVector3 cNewPos(pose->x, pose->y, 0.0);
        argos::CQuaternion cNewOrient;
        cNewOrient.FromEulerAngles(CRadians(pose->theta), CRadians(0.0), CRadians(0.0));

        // Move the physical robot model
        cFB.GetEmbodiedEntity().MoveTo(cNewPos, cNewOrient, false);
    }
    catch (...) {}
}

void ArgosRosFootbot::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr twist)
{
    // Kept alive to prevent subscriber warnings; wheel actuation is done on
    // the physical Thymio only — ARGoS position is driven by tagPoseCallback.
    Real v              = twist->linear.x  * 100.0;
    Real w              = twist->angular.z * 100.0;
    Real half_wheel_base = (9.5 / 2.0);

    leftSpeed  = v - (w * half_wheel_base);
    rightSpeed = v + (w * half_wheel_base);
}

void ArgosRosFootbot::radioTxCallback(
    const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
    // Store the message; it will be packed into a CByteArray in the next ControlStep.
    pendingRadioMsg_    = *msg;
    hasPendingRadioMsg_ = true;
}

void ArgosRosFootbot::Reset()
{
    leftSpeed  = 0.0;
    rightSpeed = 0.0;
    hasPendingRadioMsg_ = false;
    if (m_pcRABA) m_pcRABA->ClearData();
}

REGISTER_CONTROLLER(ArgosRosFootbot, "argos_ros_bot_controller")