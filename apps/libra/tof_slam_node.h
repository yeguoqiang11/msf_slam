#ifndef APPS_LIBRA_TOF_SLAM_NODE_H_
#define APPS_LIBRA_TOF_SLAM_NODE_H_

#include <memory>
#include <string>

#include "gemini/common/std_headers/std_headers.h"
#include "gemini/component/component.h"
#include "gemini/transport/domain_participant.h"
#include "modules/libra_comm/health_status_msg_generated.h"
#include "modules/libra_comm/image_msg_generated.h"
#include "modules/libra_comm/imu_msg_generated.h"
#include "modules/libra_comm/inter_proc_srvs/slam_cmd_srv.h"
#include "modules/libra_comm/inter_proc_srvs/slam_map_srv.h"
#include "modules/libra_comm/inter_proc_srvs/slam_pcl_srv.h"
#include "modules/libra_comm/odometry_msg_generated.h"
#include "modules/libra_comm/robot_mode_msg_generated.h"
#include "modules/libra_comm/slam_cmd_msg_generated.h"
#include "modules/libra_comm/slam_pose_msg_generated.h"

#include "msg_bridge.h"

namespace dm        = dreame::dm;
namespace transport = dreame::gemini::transport;
using dreame::gemini::component::Component;
using dreame::libra_msgs::HealthStatusMsgT;
using dreame::libra_msgs::ImageT;
using dreame::libra_msgs::ImuMsgT;
using dreame::libra_msgs::OdometryMsgT;
using dreame::libra_msgs::RobotModeMsgT;
using dreame::libra_msgs::SlamCmdMsgT;
using dreame::libra_msgs::SlamCmdSrv;
using dreame::libra_msgs::SlamMapSrv;
using dreame::libra_msgs::SlamPclSrv;
using dreame::libra_msgs::SlamPoseMsgT;

namespace app {

class State {
 public:
  enum { LOADING, ERROR, CHARGING, ALIGNING, RUNNING, PAUSED, LOCALIZING };

  virtual int  state()                                                 = 0;
  virtual void OnImuMsg(const std::shared_ptr<ImuMsgT>& msg)           = 0;
  virtual void OnOdometryMsg(const std::shared_ptr<OdometryMsgT>& msg) = 0;
  virtual void OnDepthImageMsg(const std::shared_ptr<ImageT>& msg)     = 0;
  virtual void OnCommand(const CmdBridge& msg)                         = 0;
  virtual void Dispatch()                                              = 0;
};

class ToFSLAMNode : public Component {
 public:
  ToFSLAMNode() {}
  ~ToFSLAMNode() {}

  // 必须实现的四个接口函数
  bool ParameterInitialize() override;
  bool InfraInitialize() override;
  bool RegisterTransport() override;
  void InternalRun() override;

  void ChangeState(std::unique_ptr<State> new_state);
  void LoadMap();

 private:

  // Tandy: 接收消息的回调函数
  // Tandy: 1. 命令
  void OnCmd(const std::shared_ptr<SlamCmdMsgT>& cmd);

  // Tandy: 2. 系统事件
  /*
   * Tandy: 扫地机健康状态，可能会影响SLAM运行策略
   * Tandy: 建议业务逻辑集中在中台，降低模块间耦合度
   */
  void OnHealthStatus(const std::shared_ptr<HealthStatusMsgT>& hmsg);

  /*
   * Tandy: 扫地机运行模式，可能会影响SLAM运行策略
   */
  void OnRobotMode(const std::shared_ptr<RobotModeMsgT>& robot_mode);

  // Tandy: 3. 传感器消息
  void OnImu(const std::shared_ptr<ImuMsgT>& msg);
  void OnOdometry(const std::shared_ptr<OdometryMsgT>& msg);
  void OnDepthImage(const std::shared_ptr<ImageT>& msg);

  // Tandy: 4. 查询
  /*
   * Tandy: 请求指定ROI的地图
   * Tandy: 返回网格地图，网格信息为uint8, 内容待定
   */
  void OnMapRequest(const std::shared_ptr<SlamMapSrv::Request>& request,
                    std::shared_ptr<SlamMapSrv::Response>&      respond);

  /*
   * Tandy: Request和Response都是slam_cmd_msg.fbs中定义的SlamCmdMsg
   * 这会让人困惑
   * BTW, 目前没有相关的调用，与之对应的cmd_server_并没有初始化，为什么？
   */
  // TODO(Tandy): 搞清楚需要响应哪些查询
  dm::optional<SlamCmdSrv::Response> OnSlamCmdRequest(
      const std::shared_ptr<SlamCmdSrv::Request>& slam_cmd_msg);

  // TODO(Tandy): 搞清楚Request里面的int代表什么含义
  dm::optional<SlamPclSrv::Response> OnSlamPclRequest(
      const std::shared_ptr<SlamPclSrv::Request>& slam_pcl_request);

  // subscribers
  transport::Subscriber<OdometryMsgT>::SharedPtr odometry_sub_;
  transport::Subscriber<ImuMsgT>::SharedPtr      imu_sub_;
  transport::Subscriber<ImageT>::SharedPtr       depth_sub_;

  transport::Subscriber<SlamCmdMsgT>::SharedPtr      cmd_sub_;
  transport::Subscriber<RobotModeMsgT>::SharedPtr    robot_mode_sub_;
  transport::Subscriber<HealthStatusMsgT>::SharedPtr health_status_sub_;

  transport::Service<SlamCmdSrv>::SharedPtr query_srv_;
  transport::Service<SlamMapSrv>::SharedPtr map_srv_;
  transport::Service<SlamPclSrv>::SharedPtr pcl_srv_;

  // publishers
  transport::Publisher<SlamPoseMsgT>::SharedPtr pose_pub_ = nullptr;
  // TODO(Tandy): 发布其他消息

  std::unique_ptr<State> state_;
};
}  // namespace app
#endif  // APPS_LIBRA_TOF_SLAM_NODE_H_
