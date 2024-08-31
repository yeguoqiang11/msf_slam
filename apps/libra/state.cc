// state implementation
// Copyright 2021 Dreame Co.Ltd. All rights reserved.

#include <memory>
#include <numeric>

#include "gemini/transport/libra_log.h"
#include "modules/libra_comm/slam_cmd_msg_generated.h"
#include "state.h"

namespace msgs = dreame::libra_msgs;

// TODO(Tandy): implementation
namespace app {

Error::Error(ToFSLAMNode* tn) : context_(tn) {}
void Error::OnCommand(const CmdBridge& msg) {
  switch (msg.type) {
    case msgs::SlamCmd_SAVE_MAP: {
      break;
    }
    case SLAM_GOTO_CHARGE: {
      std::unique_ptr<State> new_state(new Charging(context_));
      context_->ChangeState(std::move(new_state));
      break;
    }
    case msgs::SlamCmd_RESTART: {
      std::unique_ptr<State> new_state(new Aligning(context_));
      context_->ChangeState(std::move(new_state));
      break;
    }
    case msgs::SlamCmd_RELOC_MODE: {
      std::unique_ptr<State> new_state(new Running(context_));
      context_->ChangeState(std::move(new_state));
      break;
    }
    default: {
      INFO("Illegal command %d during ERROR.", msg.type);
      break;
    }
  }
}

Loading::Loading(ToFSLAMNode* tn) : Error(tn) { loading_fut_ = loading_promise_.get_future(); }

void Loading::Dispatch() {
  context_->LoadMap();
  loading_promise_.set_value();
}

void Loading::OnCommand(const CmdBridge& msg) {
  loading_fut_.wait();
  Error::OnCommand(msg);
}

Charging::Charging(ToFSLAMNode* tn) : context_(tn) {}
void Charging::OnImuMsg(const std::shared_ptr<ImuMsgT>& msg) {}
void Charging::OnOdometryMsg(const std::shared_ptr<OdometryMsgT>& msg) {}
void Charging::OnDepthImageMsg(const std::shared_ptr<ImageT>& msg) {}
void Charging::Dispatch() {}
void Charging::OnCommand(const CmdBridge& msg) {
  switch (msg.type) {
    case msgs::SlamCmd_SAVE_MAP: {
      break;
    }
    case SLAM_BREAK: {
      std::unique_ptr<State> new_state(new Error(context_));
      context_->ChangeState(std::move(new_state));
      break;
    }
    case SLAM_GOTO_CHARGE: {
      break;
    }
    case SLAM_LEAVE_CHARGE: {
      break;
    }
    default: {
      break;
    }
  }
}

Aligning::Aligning(ToFSLAMNode* tn) : context_(tn) {}
void Aligning::OnImuMsg(const std::shared_ptr<ImuMsgT>& msg) {}
void Aligning::OnOdometryMsg(const std::shared_ptr<OdometryMsgT>& msg) {}
void Aligning::OnDepthImageMsg(const std::shared_ptr<ImageT>& msg) {}
void Aligning::Dispatch() {}
void Aligning::OnCommand(const CmdBridge& msg) {
  switch (msg.type) {
    case SLAM_GOTO_CHARGE: {
      std::unique_ptr<State> new_state(new Charging(context_));
      context_->ChangeState(std::move(new_state));
      break;
    }
    case msgs::SlamCmd_PAUSE: {
      std::unique_ptr<State> new_state(new Paused(context_, ALIGNING));
      context_->ChangeState(std::move(new_state));
      break;
    }
    case SLAM_BREAK: {
      break;
    }
    default: {
      INFO("Illegal command %d during Aligning.", msg.type);
      break;
    }
  }
}

Running::Running(ToFSLAMNode* tn) : context_(tn) {}
void Running::OnImuMsg(const std::shared_ptr<ImuMsgT>& msg) {}
void Running::OnOdometryMsg(const std::shared_ptr<OdometryMsgT>& msg) {}
void Running::OnDepthImageMsg(const std::shared_ptr<ImageT>& msg) {}
void Running::Dispatch() {}
void Running::OnCommand(const CmdBridge& msg) {
  switch (msg.type) {
    case msgs::SlamCmd_PAUSE: {
      std::unique_ptr<State> new_state(new Paused(context_, RUNNING));
      context_->ChangeState(std::move(new_state));
      break;
    }
    case SLAM_GOTO_CHARGE: {
      std::unique_ptr<State> new_state(new Charging(context_));
      context_->ChangeState(std::move(new_state));
      break;
    }
    case SLAM_BREAK: {
      std::unique_ptr<State> new_state(new Error(context_));
      context_->ChangeState(std::move(new_state));
      break;
    }
    default: {
      INFO("Illegal command %d during Running.", msg.type);
      break;
    }
  }
}

Paused::Paused(ToFSLAMNode* tn, int target_state) : context_(tn), target_state_(target_state) {}
void Paused::OnImuMsg(const std::shared_ptr<ImuMsgT>& msg) {}
void Paused::OnOdometryMsg(const std::shared_ptr<OdometryMsgT>& msg) {}
void Paused::OnDepthImageMsg(const std::shared_ptr<ImageT>& msg) {}
void Paused::Dispatch() {}
void Paused::OnCommand(const CmdBridge& msg) {
  switch (msg.type) {
    case msgs::SlamCmd_SAVE_MAP: {
      break;
    }
    case SLAM_BREAK: {
      std::unique_ptr<State> new_state(new Error(context_));
      context_->ChangeState(std::move(new_state));
      break;
    }
    case msgs::SlamCmd_RESTART: {
      std::unique_ptr<State> new_state(new Aligning(context_));
      context_->ChangeState(std::move(new_state));
      break;
    }
    case msgs::SlamCmd_RESUME: {
      std::unique_ptr<State> new_state;
      if (target_state_ == ALIGNING) {
        new_state.reset(new Aligning(context_));
      } else if (target_state_ == RUNNING) {
        new_state.reset(new Running(context_));
      }
      if (new_state) {
        context_->ChangeState(std::move(new_state));
      } else {
        ERR("!!");
      }
      break;
    }
    case msgs::SlamCmd_RELOC_MODE: {
      std::unique_ptr<State> new_state(new Running(context_));
      context_->ChangeState(std::move(new_state));
      break;
    }
    case SLAM_GOTO_CHARGE: {
      std::unique_ptr<State> new_state(new Charging(context_));
      context_->ChangeState(std::move(new_state));
      break;
    }
    default: {
      INFO("Illegal command %d during Paused.", msg.type);
      break;
    }
  }
}

}  // namespace app
