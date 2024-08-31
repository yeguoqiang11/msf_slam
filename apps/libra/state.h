#ifndef APPS_LIBRA_STATE_H_
#define APPS_LIBRA_STATE_H_

#include <future>
#include <memory>
#include "tof_slam_node.h"

namespace app {

class Error : public State {
 public:
  explicit Error(ToFSLAMNode* context);
  virtual int  state() { return ERROR; }
  virtual void OnImuMsg(const std::shared_ptr<ImuMsgT>& msg) {}
  virtual void OnOdometryMsg(const std::shared_ptr<OdometryMsgT>& msg) {}
  virtual void OnDepthImageMsg(const std::shared_ptr<ImageT>& msg) {}
  virtual void OnCommand(const CmdBridge& msg);
  virtual void Dispatch() {}

 protected:
  ToFSLAMNode* context_;
};

class Loading : public Error {
 public:
  explicit Loading(ToFSLAMNode* context);
  int  state() override { return LOADING; }
  void OnCommand(const CmdBridge& msg) override;
  void Dispatch() override;

 private:
  std::promise<void> loading_promise_;
  std::future<void> loading_fut_;
};

class Charging : public State {
 public:
  explicit Charging(ToFSLAMNode* context);
  virtual int  state() { return CHARGING; }
  virtual void OnImuMsg(const std::shared_ptr<ImuMsgT>& msg);
  virtual void OnOdometryMsg(const std::shared_ptr<OdometryMsgT>& msg);
  virtual void OnDepthImageMsg(const std::shared_ptr<ImageT>& msg);
  virtual void OnCommand(const CmdBridge& msg);
  virtual void Dispatch();

 private:
  ToFSLAMNode* context_;
};

class Aligning : public State {
 public:
  explicit Aligning(ToFSLAMNode* context);
  virtual int  state() { return ALIGNING; }
  virtual void OnImuMsg(const std::shared_ptr<ImuMsgT>& msg);
  virtual void OnOdometryMsg(const std::shared_ptr<OdometryMsgT>& msg);
  virtual void OnDepthImageMsg(const std::shared_ptr<ImageT>& msg);
  virtual void OnCommand(const CmdBridge& msg);
  virtual void Dispatch();

 private:
  ToFSLAMNode* context_;
};

class Running : public State {
 public:
  explicit Running(ToFSLAMNode* context);
  virtual int  state() { return RUNNING; }
  virtual void OnImuMsg(const std::shared_ptr<ImuMsgT>& msg);
  virtual void OnOdometryMsg(const std::shared_ptr<OdometryMsgT>& msg);
  virtual void OnDepthImageMsg(const std::shared_ptr<ImageT>& msg);
  virtual void OnCommand(const CmdBridge& msg);
  virtual void Dispatch();

 private:
  ToFSLAMNode* context_;
};

class Paused : public State {
 public:
  explicit Paused(ToFSLAMNode* context, int target_state);
  virtual int  state() { return PAUSED; }
  virtual void OnImuMsg(const std::shared_ptr<ImuMsgT>& msg);
  virtual void OnOdometryMsg(const std::shared_ptr<OdometryMsgT>& msg);
  virtual void OnDepthImageMsg(const std::shared_ptr<ImageT>& msg);
  virtual void OnCommand(const CmdBridge& msg);
  virtual void Dispatch();

 private:
  ToFSLAMNode* context_;
  int          target_state_;
};

}  // namespace app

#endif  // APPS_LIBRA_STATE_H_
