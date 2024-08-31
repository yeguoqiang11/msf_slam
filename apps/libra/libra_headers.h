#ifndef MULTI_SENSOR_SLAM_APPS_ONLINE_LIBRA_HEADERS_H_
#define MULTI_SENSOR_SLAM_APPS_ONLINE_LIBRA_HEADERS_H_
/////////////////////////////////////////////////////////////////////
//主题名称
#include "modules/libra_comm/topic_declare.h"
/////////////////////////////////////////////////////////////////////
//配置文件
#include "modules/libra_comm/params/proto/node_ldslam.pb.h"
/////////////////////////////////////////////////////////////////////
//进程间通信消息格式
#include "gemini/transport/std_fbs/int_generated.h"
#include "modules/libra_comm/example_msg_generated.h"
#include "modules/libra_comm/exp_reloc_cmd_msg_generated.h"
#include "modules/libra_comm/health_status_msg_generated.h"
#include "modules/libra_comm/imu_msg_generated.h"
#include "modules/libra_comm/json_msg_generated.h"
#include "modules/libra_comm/lds_type_msg_generated.h"
#include "modules/libra_comm/odometry_msg_generated.h"
#include "modules/libra_comm/pointcloud_msg_generated.h"
#include "modules/libra_comm/pointcloud_with_pose_and_time_msg_generated.h"
#include "modules/libra_comm/pointcloud_with_pose_msg_generated.h"
#include "modules/libra_comm/reloc_res_msg_generated.h"
#include "modules/libra_comm/robot_mode_msg_generated.h"
#include "modules/libra_comm/scan_msg_generated.h"
#include "modules/libra_comm/image_msg_generated.h"
#include "modules/libra_comm/slam_cmd_msg_generated.h"
#include "modules/libra_comm/slam_map_msg_generated.h"
#include "modules/libra_comm/slam_pose_msg_generated.h"
#include "modules/libra_comm/slam_status_msg_generated.h"
#include "modules/libra_comm/slam_upload_pkt_msg_generated.h"
/////////////////////////////////////////////////////////////////////
//服务消息
#include "modules/libra_comm/inter_proc_srvs/json_srv.h"
#include "modules/libra_comm/inter_proc_srvs/slam_cmd_srv.h"
#include "modules/libra_comm/inter_proc_srvs/slam_map_srv.h"
#include "modules/libra_comm/inter_proc_srvs/slam_pcl_srv.h"
/////////////////////////////////////////////////////////////////////
//框架依赖头文件
#include "gemini/class_loader/class_register_macro.h"
#include "gemini/common/file/file_or_contents.h"
#include "gemini/common/std_headers/std_headers.h"
#include "gemini/component/component.h"
#include "gemini/transport/domain_participant.h"
/////////////////////////////////////////////////////////////////////
namespace dmsg     = dreame::libra_msgs;
namespace ddm      = dreame::dm;
namespace dmsg_std = dreame::std_msgs;
namespace dtrans   = dreame::gemini::transport;
namespace dcompo   = dreame::gemini::component;
namespace dslam    = dreame::libra_node_slam;
namespace dfile    = dreame::gemini::file;
/////////////////////////////////////////////////////////////////////
#define PUBLISH_LOCAL_POINTS 1
/////////////////////////////////////////////////////////////////////
/*
#libra编译
bazel build -c opt --config=openwrt-platform //modules/libra_comm/artifact/release:ldslam

#libra成果物上传
cp -f bazel-bin/modules/libra_comm/artifact/release/ldslam.tar.gz bazel-tmp/
adb.exe push bazel-tmp/ldslam.tar.gz /data/libra

#libra 运行
./gemini/component/component_manager_main
-config_file=./modules/libra_comm/component_manager_configs/middleware_manager.config
./gemini/component/component_manager_main
-config_file=./modules/libra_comm/component_manager_configs/ldslam_manager.config
*/
/////////////////////////////////////////////////////////////////////
#endif
