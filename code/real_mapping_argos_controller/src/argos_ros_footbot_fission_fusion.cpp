#include "argos_ros_footbot_fission_fusion.h"
#include <cmath>
#include <cstring>
using namespace std;
using std::placeholders::_1;

// Constructor: Initializes raw pointers to NULL and sets pending broadcast flag to false
ArgosRosFootbot::ArgosRosFootbot() : m_pcProximity(NULL),
                                     m_pcRABS(NULL),
                                     m_pcRadioTx(NULL),
                                     m_pcRadioRx(NULL),
                                     hasPendingRadioMsg_(false) {}

ArgosRosFootbot::~ArgosRosFootbot() {}

void ArgosRosFootbot::Init(TConfigurationNode &t_node)
{
    // Ensure the ROS 2 client library is active
    int argc = 0;
    char **argv = nullptr;
    if (!rclcpp::ok()) rclcpp::init(argc, argv);

    // Create the ROS 2 node bound to this specific robot instance ID
    std::string node_name = GetId() + "_argos3_ros_bridge";
    nodeHandle_ = std::make_shared<rclcpp::Node>(node_name);

    // Parse robot ID number from entity name (e.g., "bot7" -> 7) for AprilTag mapping
    int tag_id = 1; 
    std::string bot_name = GetId();
    if (bot_name.find("bot") != std::string::npos) {
        tag_id = std::stoi(bot_name.substr(3)); 
    }

    // ── Topic Name Formatting ────────────────────────────────────────────────
    stringstream vProxTopic, neighborTopic, tagTopic;
    stringstream radioRxTopic, radioTxTopic;

    vProxTopic   << "/" << GetId() << "/virtual_prox";
    neighborTopic << "/" << GetId() << "/neighbors";
    tagTopic     << "/tag_tracker/pose/tag_" << tag_id;
    radioRxTopic << "/" << GetId() << "/radio_rx";   // ARGoS → ROS  (received)
    radioTxTopic << "/" << GetId() << "/radio_tx";   // ROS → ARGoS  (to send)

    // ── ROS 2 Publishers ─────────────────────────────────────────────────────
    virtualProxPublisher_ = nodeHandle_->create_publisher<std_msgs::msg::Float64MultiArray>(
        vProxTopic.str(), 10);
    neighborPublisher_ = nodeHandle_->create_publisher<std_msgs::msg::Float64MultiArray>(
        neighborTopic.str(), 10);
    radioRxPublisher_ = nodeHandle_->create_publisher<std_msgs::msg::Float64MultiArray>(
        radioRxTopic.str(), 10);

    // ── ROS 2 Subscribers ────────────────────────────────────────────────────
    auto best_effort_qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
    tagPoseSubscriber_ = nodeHandle_->create_subscription<geometry_msgs::msg::Pose2D>(
        tagTopic.str(), best_effort_qos,
        std::bind(&ArgosRosFootbot::tagPoseCallback, this, _1));

    radioTxSubscriber_ = nodeHandle_->create_subscription<std_msgs::msg::Float64MultiArray>(
        radioTxTopic.str(), 10,
        std::bind(&ArgosRosFootbot::radioTxCallback, this, _1));

    // ── Bind ARGoS Hardware/Plugin Interfaces ────────────────────────────────
    m_pcProximity = GetSensor<CCI_FootBotProximitySensor>("footbot_proximity");
    m_pcRABS      = GetSensor<CCI_RangeAndBearingSensor>("range_and_bearing");
    m_pcRadioTx   = GetActuator<CCI_SimpleRadiosActuator>("simple_radios");
    m_pcRadioRx   = GetSensor<CCI_SimpleRadiosSensor>("simple_radios");

    Reset();
}

void ArgosRosFootbot::ControlStep()
{
    // Process incoming ROS 2 events
    rclcpp::spin_some(nodeHandle_);

    // ── 1. Publish Proximity Readings → ROS 2 ────────────────────────────────
    const CCI_FootBotProximitySensor::TReadings &tProxReads = m_pcProximity->GetReadings();
    std_msgs::msg::Float64MultiArray raw_prox_msg;
    raw_prox_msg.data.reserve(tProxReads.size());
    for (size_t i = 0; i < tProxReads.size(); ++i)
        raw_prox_msg.data.push_back(static_cast<double>(tProxReads[i].Value));
    virtualProxPublisher_->publish(raw_prox_msg);

    // ── 2. Publish Range-and-Bearing Neighbor Positions → ROS 2 ───────────────
    const CCI_RangeAndBearingSensor::TReadings &tRabReads = m_pcRABS->GetReadings();
    std_msgs::msg::Float64MultiArray neighbor_msg;
    for (size_t i = 0; i < tRabReads.size(); ++i) {
        neighbor_msg.data.push_back(tRabReads[i].Range);
        neighbor_msg.data.push_back(tRabReads[i].HorizontalBearing.GetValue());
    }
    neighborPublisher_->publish(neighbor_msg);

    // ── 3. Transmit Pending Radio Message (ROS 2 → ARGoS Medium) ─────────────
    // Pack double-precision values into 8-byte raw byte streams for transmission
    if (hasPendingRadioMsg_) {
        CByteArray cData;
        for (double val : pendingRadioMsg_.data) {
            uint64_t raw;
            memcpy(&raw, &val, sizeof(double));
            for (int b = 0; b < 8; ++b)
                cData << static_cast<UInt8>((raw >> (b * 8)) & 0xFF);
        }
        m_pcRadioTx->GetInterfaces()[0].Messages.push_back(cData);
        hasPendingRadioMsg_ = false;
    }

    // ── 4. Forward Received Radio Messages (ARGoS Medium → ROS 2) ────────────
    // Unpack received 8-byte streams back into double-precision float arrays
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
}

// ── Callbacks ────────────────────────────────────────────────────────────────

void ArgosRosFootbot::tagPoseCallback(const geometry_msgs::msg::Pose2D::SharedPtr pose)
{
    // Math Safety Net: Reject invalid or corrupted position matrices
    if (std::isnan(pose->x) || std::isnan(pose->y) || std::isnan(pose->theta) ||
        std::isinf(pose->x) || std::isinf(pose->y) || std::isinf(pose->theta))
    {
        return;
    }

    // Arena Boundary Check: Ignore poses outside the physical tracking arena
    if (pose->x < -0.35 || pose->x > 3.15 || 
        pose->y < -0.35 || pose->y > 2.55) 
    {
        return; // Position is outside the simulator map! Ignore this camera frame.
    }

    // Update simulated entity position and orientation in ARGoS simulation space
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

void ArgosRosFootbot::radioTxCallback(
    const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
    // Buffer ROS message data to be dispatched during the next ControlStep iteration
    pendingRadioMsg_    = *msg;
    hasPendingRadioMsg_ = true;
}

void ArgosRosFootbot::Reset()
{
    hasPendingRadioMsg_ = false;
}

REGISTER_CONTROLLER(ArgosRosFootbot, "argos_ros_bot_controller")