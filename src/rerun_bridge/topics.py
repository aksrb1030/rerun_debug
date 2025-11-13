"""Topic configuration for the ROS 2 to Rerun bridge."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable, List, Optional


# 이 데이터 클래스는 구독할 토픽의 이름, 메시지 타입, QoS를 보관합니다.
@dataclass(frozen=True)
class TopicConfig:
    topic: str
    type_string: str
    qos: Optional[str] = None


# 이 함수는 단위 이름에 따라 구독할 ROS 2 토픽 목록을 생성합니다.
def build_topic_list(unit_name: str) -> List[TopicConfig]:
    sensor_qos = "sensor"

    prefixed: Iterable[TopicConfig] = [
        TopicConfig(f"/{unit_name}/can/board_status", "pkr_msgs/msg/EcuSTAT"),
        TopicConfig(f"/{unit_name}/can/eps_control", "pkr_msgs/msg/MotorCMD"),
        TopicConfig(f"/{unit_name}/can/eps_status", "pkr_msgs/msg/MotorSTAT"),
        TopicConfig(f"/{unit_name}/ctrl/task_request", "rtm_msgs/msg/IpkcComm"),
        TopicConfig(f"/{unit_name}/dkc/task_request", "rtm_msgs/msg/IpkcComm"),
        TopicConfig(f"/{unit_name}/driving_mode", "std_msgs/msg/UInt16"),
        TopicConfig(f"/{unit_name}/err_hdlr/cmd", "std_msgs/msg/UInt8"),
        TopicConfig(f"/{unit_name}/feedforward/motion_status", "dual_msgs/msg/MotionStatus"),
        TopicConfig(f"/{unit_name}/filt_rel_pose_1", "geometry_msgs/msg/Pose"),
        TopicConfig(f"/{unit_name}/front/ground_coeff", "nav_msgs/msg/Odometry"),
        TopicConfig(f"/{unit_name}/imu", "sensor_msgs/msg/Imu", sensor_qos),
        TopicConfig(f"/{unit_name}/itf/task_request", "rtm_msgs/msg/ItfComm"),
        TopicConfig(f"/{unit_name}/mon_cmd", "nav_msgs/msg/Odometry"),
        TopicConfig(f"/{unit_name}/monitoring/obstacle", "nav_msgs/msg/Odometry"),
        TopicConfig(f"/{unit_name}/monitoring/task", "nav_msgs/msg/Odometry"),
        TopicConfig(f"/{unit_name}/motion_Cmd", "geometry_msgs/msg/Twist"),
        TopicConfig(f"/{unit_name}/obstacle_detection/critical_path", "nav_msgs/msg/Path"),
        TopicConfig(
            f"/{unit_name}/obstacle_detection/obstacle_points",
            "sensor_msgs/msg/PointCloud2",
            sensor_qos,
        ),
        TopicConfig(f"/{unit_name}/obstacle_detection/warning_path", "nav_msgs/msg/Path"),
        TopicConfig(f"/{unit_name}/odom", "nav_msgs/msg/Odometry"),
        TopicConfig(f"/{unit_name}/other_err_hdlr/error", "std_msgs/msg/UInt64"),
        TopicConfig(f"/{unit_name}/other_obs_dualM", "geometry_msgs/msg/Twist"),
        TopicConfig(f"/{unit_name}/other_obs_dual_motion", "geometry_msgs/msg/Twist"),
        TopicConfig(f"/{unit_name}/other_tk_ctrl/init", "geometry_msgs/msg/PointStamped"),
        TopicConfig(f"/{unit_name}/pose_COG", "nav_msgs/msg/Odometry"),
        TopicConfig(f"/{unit_name}/pose_car", "geometry_msgs/msg/PoseStamped"),
        TopicConfig(f"/{unit_name}/rear/ground_coeff", "nav_msgs/msg/Odometry"),
        TopicConfig(f"/{unit_name}/rjh/task_request", "pkr_msgs/msg/RobotMissionTmp"),
        TopicConfig(f"/{unit_name}/rjh/task_response", "std_msgs/msg/String"),
        TopicConfig(f"/{unit_name}/rtm/stat", "rtm_msgs/msg/RtmStat"),
        TopicConfig(f"/{unit_name}/rtm/task_result", "pkr_msgs/msg/Tm2ActrRSLT"),
        TopicConfig(f"/{unit_name}/rtm/task_result_response", "std_msgs/msg/String"),
        TopicConfig(
            f"/{unit_name}/target_and_now_wassy_angle",
            "std_msgs/msg/Float64MultiArray",
        ),
        TopicConfig(f"/{unit_name}/this/obs_dualM", "geometry_msgs/msg/Twist"),
        TopicConfig(f"/{unit_name}/this/obs_dual_motion", "geometry_msgs/msg/Twist"),
        TopicConfig(f"/{unit_name}/this/pose_COG", "nav_msgs/msg/Odometry"),
        TopicConfig(f"/{unit_name}/this/tk_ctrl/init", "geometry_msgs/msg/PointStamped"),
        TopicConfig(f"/{unit_name}/spatial_logmsg", "rcl_interfaces/msg/Log"),
    ]

    globals_: Iterable[TopicConfig] = [
        TopicConfig("/diagnostics_agg", "diagnostic_msgs/msg/DiagnosticArray"),
        TopicConfig("/hesai/pandar", "sensor_msgs/msg/PointCloud2", sensor_qos),
        TopicConfig("/right/hesai/pandar", "sensor_msgs/msg/PointCloud2", sensor_qos),
    ]

    return [*prefixed, *globals_]
