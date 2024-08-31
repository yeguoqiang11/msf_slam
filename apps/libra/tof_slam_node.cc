#include "tof_slam_node.h"
#include "gemini/class_loader/class_register_macro.h"
#include "modules/libra_comm/topic_declare.h"
#include "gemini/transport/libra_log.h"

CLASS_LOADER_REGISTER_CLASS(app::ToFSLAMNode,
                            dreame::gemini::component::ComponentBase);

using dreame::libra_msgs::DepthImageTopic;
using dreame::libra_msgs::OdometryMsgTopic;
using dreame::libra_msgs::ImuMsgTopic;
using dreame::libra_msgs::SlamCmdTopic;
using dreame::libra_msgs::RobotModeMsgTopic;
using dreame::libra_msgs::HealthStatusMsgTopic;
using dreame::libra_msgs::SlamSrvMapRequestTopic;
using dreame::libra_msgs::SlamPoseMsgTopic;

namespace app {

bool ToFSLAMNode::ParameterInitialize() { return true; }
bool ToFSLAMNode::InfraInitialize() { return true; }
bool ToFSLAMNode::RegisterTransport() {
  auto& nh = *transport::DomainParticipant::Instance();

  depth_sub_ = nh.CreateSubscriber<ImageT>(
      dreame::libra_msgs::DepthImageTopic, 25, std::bind(&ToFSLAMNode::OnDepthImage, this, std::placeholders::_1));

  odometry_sub_ = nh.CreateSubscriber<OdometryMsgT>(
      dreame::libra_msgs::OdometryMsgTopic, 250,
      std::bind(&ToFSLAMNode::OnOdometry, this, std::placeholders::_1));

  imu_sub_ = nh.CreateSubscriber<ImuMsgT>(
      dreame::libra_msgs::ImuMsgTopic, 500, std::bind(&ToFSLAMNode::OnImu, this, std::placeholders::_1));

  cmd_sub_ = nh.CreateSubscriber<SlamCmdMsgT>(
      dreame::libra_msgs::SlamCmdTopic, 10,
      std::bind(&ToFSLAMNode::OnCmd, this, std::placeholders::_1));

  robot_mode_sub_ = nh.CreateSubscriber<RobotModeMsgT>(
      dreame::libra_msgs::RobotModeMsgTopic, 10,
      std::bind(&ToFSLAMNode::OnRobotMode, this, std::placeholders::_1));

  health_status_sub_ = nh.CreateSubscriber<HealthStatusMsgT>(
      HealthStatusMsgTopic, 10,
      std::bind(&ToFSLAMNode::OnHealthStatus, this, std::placeholders::_1));

  //初始化：注册服务：map请求
  map_srv_ = nh.CreateServer<SlamMapSrv>(
      SlamSrvMapRequestTopic, std::bind(&ToFSLAMNode::OnMapRequest, this,
                                              std::placeholders::_1, std::placeholders::_2));


  //初始化：发布消息：slam位置
  pose_pub_ = nh.CreatePublisher<SlamPoseMsgT>(SlamPoseMsgTopic);

  return true;
}

void ToFSLAMNode::InternalRun() {
  //空运转线程
  INFO("ToFSLAMNode start internal running %d %d %d...", 3, 2, 1);
  while (dreame::gemini::OK()) {
    sleep(1);
  }
}

//////////////////////////////////////////////////////////////////////
//机器人的工作模式切换，对于slam来说，主要用于判断是否在充电、是否要暂停等
//////////////////////////////////////////////////////////////////////
void ToFSLAMNode::OnRobotMode(const std::shared_ptr<RobotModeMsgT>& robot_mode_msg) {
  auto robot_mode = robot_mode_msg->mode;

  //判断机器人是否在充电中
  if (robot_mode == dreame::libra_msgs::RobotMode_ChargingMode) {
  }

  //判断机器人是否暂停
  if (robot_mode == dreame::libra_msgs::RobotMode_PauseAndStopMode || robot_mode == dreame::libra_msgs::RobotMode_StandbyMode ||
      robot_mode == dreame::libra_msgs::RobotMode_ErrRepotMode) {
    //slam_system_node_->CtrlPause();
  }
}

//////////////////////////////////////////////////////////////////////
//机器人的健康状态，对于slam来说，用于判断机器人是否被搬起
//////////////////////////////////////////////////////////////////////
void ToFSLAMNode::OnHealthStatus(const std::shared_ptr<HealthStatusMsgT>& hmsg) {
  //bool prev_is_hanging = time_in_hanging_ > 0;
  //bool curr_is_hanging = false;
  //if (hmsg->check.size() > (int)(dmsg::HealthType_MOVE) &&
      //hmsg->check[dmsg::HealthType_MOVE] != dmsg::HealthStatus_NORMAL) {
    //curr_is_hanging = true;
  //}

  //if (!prev_is_hanging && curr_is_hanging) {
    //time_in_hanging_ = time_utils::NowTimeMsecond();
    ////被搬起，slam暂停
    //slam_system_node_->CtrlPause();
  //}
  //if (prev_is_hanging && !curr_is_hanging) {
    //time_in_hanging_ = 0;
  //}
}

}  // namespace app
