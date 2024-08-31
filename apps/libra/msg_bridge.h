// bridge between libra_comm and internal msg
// Copyright 2021 Dreame Co.Ltd. All rights reserved.

#ifndef APPS_LIBRA_MSG_BRIDGE_H_
#define APPS_LIBRA_MSG_BRIDGE_H_

#include <vector>
#include "modules/libra_comm/slam_cmd_msg_generated.h"

using dreame::libra_msgs::SlamCmd_MAX;

namespace app {

typedef enum {
  SLAM_BREAK = SlamCmd_MAX + 1,
  SLAM_GOTO_CHARGE,
  SLAM_LEAVE_CHARGE
} ExtendedCmd;

struct CmdBridge {
  int type;
  std::vector<int> args;
};

}  // namespace app

#endif  // APPS_LIBRA_MSG_BRIDGE_H_
