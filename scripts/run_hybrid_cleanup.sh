#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
DOCKER_COMPOSE="docker compose"
command -v docker-compose >/dev/null 2>&1 && DOCKER_COMPOSE="docker-compose"

ROS_MASTER_URI_DEFAULT="http://127.0.0.1:11311"
ROS_IP_DEFAULT="127.0.0.1"
PLACE_BIN_COLOR="${PLACE_BIN_COLOR:-green}"
GRASP_POSE_SOURCE="${GRASP_POSE_SOURCE:-yolo}"
START_GRASP_SERVICE="${START_GRASP_SERVICE:-false}"
SCENE_WIDE_NON_CUBE_RANKING="${SCENE_WIDE_NON_CUBE_RANKING:-false}"
DEBUG_NON_CUBE_ONLY="${DEBUG_NON_CUBE_ONLY:-false}"
EXECUTE_AFTER_SCENE_RANKING="${EXECUTE_AFTER_SCENE_RANKING:-true}"
SCENE_RANK_MIN_POSE_Z="${SCENE_RANK_MIN_POSE_Z:-0.10}"
SCENE_RANK_MIN_TOP_SCORE="${SCENE_RANK_MIN_TOP_SCORE:-0.0}"
NON_CUBE_BBOX_WIDTH_MARGIN="${NON_CUBE_BBOX_WIDTH_MARGIN:--0.18}"
CUBE_NOMINAL_GRASP_WIDTH_M="${CUBE_NOMINAL_GRASP_WIDTH_M:-0.0278}"
NON_CUBE_FIXED_WIDTH_MULTIPLIER="${NON_CUBE_FIXED_WIDTH_MULTIPLIER:-2.5}"
CUBE_PLACE_JOINTS="${CUBE_PLACE_JOINTS:-[0.3000,0.7006,1.5000,-2.5000,0.0002,0.0000]}"
EXCLUDE_CUBES_ON_SCALE="${EXCLUDE_CUBES_ON_SCALE:-true}"
SCALE_FILTER_WORLD_FRAME="${SCALE_FILTER_WORLD_FRAME:-world}"
SCALE_FILTER_CORNER_OUTPUT_FRAME="${SCALE_FILTER_CORNER_OUTPUT_FRAME:-base_link}"
SCALE_FILTER_CORNER_WORLD_Z="${SCALE_FILTER_CORNER_WORLD_Z:-0.275}"
SCALE_FILTER_VOLUME_DOWN_M="${SCALE_FILTER_VOLUME_DOWN_M:-0.80}"
SCALE_FILTER_MIN_WORLD_Z="${SCALE_FILTER_MIN_WORLD_Z:-0.54}"
SCALE_FILTER_POLYGON_WORLD_XY="${SCALE_FILTER_POLYGON_WORLD_XY:-[[0.486,0.304],[0.627,0.185],[0.834,0.432],[0.693,0.550]]}"
PUBLISH_SCALE_FILTER_CORNERS="${PUBLISH_SCALE_FILTER_CORNERS:-true}"
SCALE_FILTER_CORNERS_TOPIC="${SCALE_FILTER_CORNERS_TOPIC:-/brain/scale_filter_corners}"
SCALE_FILTER_CORNERS_MARKER_TOPIC="${SCALE_FILTER_CORNERS_MARKER_TOPIC:-/brain/scale_filter_corners_markers}"
SCENE_PREOBSERVE_ENABLED="${SCENE_PREOBSERVE_ENABLED:-false}"
SCENE_PREOBSERVE_BASE_X_OFFSET="${SCENE_PREOBSERVE_BASE_X_OFFSET:-0.40}"
SCENE_PREOBSERVE_BASE_Y_OFFSET="${SCENE_PREOBSERVE_BASE_Y_OFFSET:-0.0}"
SCENE_PREOBSERVE_BASE_Z_OFFSET="${SCENE_PREOBSERVE_BASE_Z_OFFSET:-0.0}"
SCENE_PREOBSERVE_REFRESH_TIMEOUT="${SCENE_PREOBSERVE_REFRESH_TIMEOUT:-1.5}"
FOLLOW_LOGS=1
START_PATH_PLANNING=0

while (($#)); do
  case "$1" in
    --no-follow)
      FOLLOW_LOGS=0
      ;;
    --skip-path-planning)
      START_PATH_PLANNING=0
      ;;
    --with-path-planning)
      START_PATH_PLANNING=1
      ;;
    --bin-color)
      shift
      PLACE_BIN_COLOR="${1:-green}"
      ;;
    -h|--help)
      cat <<'USAGE'
Usage: ./scripts/run_hybrid_cleanup.sh [--bin-color green] [--no-follow] [--with-path-planning]

Assumptions:
  1. arm_gazebo is already running on http://127.0.0.1:11311
  2. brain uses hybrid grasping:
     - cube -> direct YOLO grasp pose
     - non-cube -> direct YOLO grasp pose
     - alternate cube/non-cube when both categories are selectable
  3. brain keeps picking until no selectable targets remain
  4. non-cube selection comes from direct YOLO grasp poses
  5. path_planning is optional; fallback motion control works by default
USAGE
      exit 0
      ;;
    *)
      echo "[cleanup] Unknown option: $1" >&2
      exit 1
      ;;
  esac
  shift || true
done

export ROS_MASTER_URI="${ROS_MASTER_URI:-$ROS_MASTER_URI_DEFAULT}"
export ROS_IP="${ROS_IP:-$ROS_IP_DEFAULT}"

brain_run() {
  docker run --rm --network host \
    -e ROS_MASTER_URI="$ROS_MASTER_URI" \
    -e ROS_IP="$ROS_IP" \
    --entrypoint bash \
    robocup_ur5e/brain:latest \
    -lc "$1"
}

wait_for_master() {
  timeout 2 bash -lc "echo >/dev/tcp/127.0.0.1/11311" >/dev/null 2>&1
}

wait_for_joint_states() {
  brain_run "source /opt/ros/noetic/setup.bash && source /workspace/devel/setup.bash && rostopic echo -n 1 /joint_states/header" >/dev/null
}

wait_for_topic_message() {
  local topic="$1"
  local timeout_sec="$2"
  timeout "$timeout_sec" docker run --rm --network host \
    -e ROS_MASTER_URI="$ROS_MASTER_URI" \
    -e ROS_IP="$ROS_IP" \
    --entrypoint bash \
    robocup_ur5e/brain:latest \
    -lc "source /opt/ros/noetic/setup.bash && source /workspace/devel/setup.bash && rostopic echo -n 1 ${topic}" >/dev/null
}

cleanup_old_runtime() {
  docker rm -f cleanup_brain cleanup_motion_control cleanup_yolo_cloud cleanup_yolo_json 2>/dev/null || true
  docker rm -f robocup_brain perception_grasp path_planning motion_control_full_test motion_control_alt_test brain_full_test brain_cleanup 2>/dev/null || true
  docker ps -aq --filter ancestor=robocup_ur5e/perception_yolo_gpu_native:latest | xargs -r docker rm -f >/dev/null 2>&1 || true
  docker ps -aq --filter ancestor=robocup_ur5e/brain:latest | xargs -r docker rm -f >/dev/null 2>&1 || true
  (
    cd "$REPO_ROOT"
    $DOCKER_COMPOSE stop perception_yolo 2>/dev/null || true
    $DOCKER_COMPOSE rm -f perception_yolo 2>/dev/null || true
  )
}

start_grasp() {
  (
    cd "$REPO_ROOT"
    ROS_MASTER_URI="$ROS_MASTER_URI" ROS_IP="$ROS_IP" $DOCKER_COMPOSE up -d perception_grasp
  )
}

start_path_planning() {
  docker rm -f path_planning 2>/dev/null || true
  (
    cd "$REPO_ROOT"
    ROS_MASTER_URI="$ROS_MASTER_URI" ROS_IP="$ROS_IP" $DOCKER_COMPOSE up -d path_planning
  )
}

start_yolo_cloud() {
  docker run -d \
    --name cleanup_yolo_cloud \
    --network host \
    --gpus all \
    -e ROS_MASTER_URI="$ROS_MASTER_URI" \
    -e ROS_IP="$ROS_IP" \
    -e NVIDIA_VISIBLE_DEVICES=all \
    -e NVIDIA_DRIVER_CAPABILITIES=compute,utility \
    -v "$REPO_ROOT/src/common_msgs:/workspace/src/common_msgs:ro" \
    -v "$REPO_ROOT/src/perception_yolo:/workspace/src/perception_yolo:rw" \
    -v "$REPO_ROOT/weights:/workspace/weights:rw" \
    -v "$REPO_ROOT/pointclouds:/workspace/pointclouds:rw" \
    --entrypoint bash \
    robocup_ur5e/perception_yolo_gpu_native:latest \
    -lc "source /opt/ros/noetic/setup.bash && python3 /workspace/src/perception_yolo/nodes/test_3dcloud_copy.py _model_path:=/workspace/weights/yolo/best.pt _confidence_threshold:=0.2 _cloud_topic:=/perception/yolo_bbox_instance_cloud"
}

start_yolo_json() {
  docker run -d \
    --name cleanup_yolo_json \
    --network host \
    --gpus all \
    -e ROS_MASTER_URI="$ROS_MASTER_URI" \
    -e ROS_IP="$ROS_IP" \
    -e NVIDIA_VISIBLE_DEVICES=all \
    -e NVIDIA_DRIVER_CAPABILITIES=compute,utility \
    -v "$REPO_ROOT/src/common_msgs:/workspace/src/common_msgs:ro" \
    -v "$REPO_ROOT/src/perception_yolo:/workspace/src/perception_yolo:rw" \
    -v "$REPO_ROOT/weights:/workspace/weights:rw" \
    -v "$REPO_ROOT/pointclouds:/workspace/pointclouds:rw" \
    --entrypoint bash \
    robocup_ur5e/perception_yolo_gpu_native:latest \
    -lc "source /opt/ros/noetic/setup.bash && python3 /workspace/src/perception_yolo/nodes/yolo26_seg_json_node.py _model_path:=/workspace/weights/yolo/best.pt _confidence_threshold:=0.2"
}

start_motion_control() {
  docker run -d \
    --name cleanup_motion_control \
    --network host \
    -e ROS_MASTER_URI="$ROS_MASTER_URI" \
    -e ROS_IP="$ROS_IP" \
    -v "$REPO_ROOT:/host_workspace" \
    --entrypoint bash \
    robocup_ur5e/brain:latest \
    -lc 'source /opt/ros/noetic/setup.bash && source /workspace/devel/setup.bash && export PYTHONPATH=/host_workspace/devel/lib/python3/dist-packages:$PYTHONPATH && export ROS_PACKAGE_PATH=/host_workspace/src:/workspace/src:/opt/ros/noetic/share && python3 /host_workspace/src/motion_control/nodes/motion_control_node.py'
}

prepare_brain_params() {
  brain_run "source /opt/ros/noetic/setup.bash && source /workspace/devel/setup.bash && rosparam delete /robocup_brain/preferred_target_label >/dev/null 2>&1 || true && rosparam set /robocup_brain/preferred_target_label '' && rosparam set /robocup_brain/defer_cube_until_others false && rosparam set /robocup_brain/alternate_pick_categories false && rosparam set /robocup_brain/alternate_start_category cube && rosparam set /robocup_brain/grasp_pose_source ${GRASP_POSE_SOURCE} && rosparam set /robocup_brain/cube_direct_grasp_offset_mode local_x && rosparam set /robocup_brain/scene_wide_non_cube_ranking ${SCENE_WIDE_NON_CUBE_RANKING} && rosparam set /robocup_brain/debug_non_cube_only ${DEBUG_NON_CUBE_ONLY} && rosparam set /robocup_brain/execute_after_scene_ranking ${EXECUTE_AFTER_SCENE_RANKING} && rosparam set /robocup_brain/scene_rank_min_pose_z ${SCENE_RANK_MIN_POSE_Z} && rosparam set /robocup_brain/scene_rank_min_top_score ${SCENE_RANK_MIN_TOP_SCORE} && rosparam set /robocup_brain/non_cube_bbox_width_margin ${NON_CUBE_BBOX_WIDTH_MARGIN} && rosparam set /robocup_brain/cube_nominal_grasp_width_m ${CUBE_NOMINAL_GRASP_WIDTH_M} && rosparam set /robocup_brain/non_cube_fixed_width_multiplier ${NON_CUBE_FIXED_WIDTH_MULTIPLIER} && rosparam set /robocup_brain/cube_place_joints '${CUBE_PLACE_JOINTS}' && rosparam set /robocup_brain/exclude_cubes_on_scale ${EXCLUDE_CUBES_ON_SCALE} && rosparam set /robocup_brain/scale_filter_world_frame ${SCALE_FILTER_WORLD_FRAME} && rosparam set /robocup_brain/scale_filter_corner_output_frame ${SCALE_FILTER_CORNER_OUTPUT_FRAME} && rosparam set /robocup_brain/scale_filter_corner_world_z ${SCALE_FILTER_CORNER_WORLD_Z} && rosparam set /robocup_brain/scale_filter_volume_down_m ${SCALE_FILTER_VOLUME_DOWN_M} && rosparam set /robocup_brain/scale_filter_min_world_z ${SCALE_FILTER_MIN_WORLD_Z} && rosparam set /robocup_brain/scale_filter_polygon_world_xy '${SCALE_FILTER_POLYGON_WORLD_XY}' && rosparam set /robocup_brain/publish_scale_filter_corners ${PUBLISH_SCALE_FILTER_CORNERS} && rosparam set /robocup_brain/scale_filter_corners_topic ${SCALE_FILTER_CORNERS_TOPIC} && rosparam set /robocup_brain/scale_filter_corners_marker_topic ${SCALE_FILTER_CORNERS_MARKER_TOPIC} && rosparam set /robocup_brain/scene_preobserve_enabled ${SCENE_PREOBSERVE_ENABLED} && rosparam set /robocup_brain/scene_preobserve_base_x_offset ${SCENE_PREOBSERVE_BASE_X_OFFSET} && rosparam set /robocup_brain/scene_preobserve_base_y_offset ${SCENE_PREOBSERVE_BASE_Y_OFFSET} && rosparam set /robocup_brain/scene_preobserve_base_z_offset ${SCENE_PREOBSERVE_BASE_Z_OFFSET} && rosparam set /robocup_brain/scene_preobserve_refresh_timeout ${SCENE_PREOBSERVE_REFRESH_TIMEOUT} && rosparam set /robocup_brain/place_bin_color ${PLACE_BIN_COLOR}"
}

start_brain() {
  docker run -d \
    --name cleanup_brain \
    --network host \
    -e ROS_MASTER_URI="$ROS_MASTER_URI" \
    -e ROS_IP="$ROS_IP" \
    -v "$REPO_ROOT:/host_workspace" \
    --entrypoint bash \
    robocup_ur5e/brain:latest \
    -lc "source /opt/ros/noetic/setup.bash && source /workspace/devel/setup.bash && export PYTHONPATH=/host_workspace/devel/lib/python3/dist-packages:\$PYTHONPATH && export ROS_PACKAGE_PATH=/host_workspace/src:/workspace/src:/opt/ros/noetic/share && python3 /host_workspace/src/robocup_brain/nodes/brain_node.py _grasp_pose_source:=${GRASP_POSE_SOURCE} _defer_cube_until_others:=false _alternate_pick_categories:=false _alternate_start_category:=cube _cube_direct_grasp_offset_mode:=local_x _scene_wide_non_cube_ranking:=${SCENE_WIDE_NON_CUBE_RANKING} _debug_non_cube_only:=${DEBUG_NON_CUBE_ONLY} _execute_after_scene_ranking:=${EXECUTE_AFTER_SCENE_RANKING} _scene_rank_min_pose_z:=${SCENE_RANK_MIN_POSE_Z} _scene_rank_min_top_score:=${SCENE_RANK_MIN_TOP_SCORE} _non_cube_bbox_width_margin:=${NON_CUBE_BBOX_WIDTH_MARGIN} _cube_nominal_grasp_width_m:=${CUBE_NOMINAL_GRASP_WIDTH_M} _non_cube_fixed_width_multiplier:=${NON_CUBE_FIXED_WIDTH_MULTIPLIER} _cube_place_joints:=${CUBE_PLACE_JOINTS} _exclude_cubes_on_scale:=${EXCLUDE_CUBES_ON_SCALE} _scale_filter_world_frame:=${SCALE_FILTER_WORLD_FRAME} _scale_filter_corner_output_frame:=${SCALE_FILTER_CORNER_OUTPUT_FRAME} _scale_filter_corner_world_z:=${SCALE_FILTER_CORNER_WORLD_Z} _scale_filter_volume_down_m:=${SCALE_FILTER_VOLUME_DOWN_M} _scale_filter_min_world_z:=${SCALE_FILTER_MIN_WORLD_Z} _scale_filter_polygon_world_xy:=${SCALE_FILTER_POLYGON_WORLD_XY} _publish_scale_filter_corners:=${PUBLISH_SCALE_FILTER_CORNERS} _scale_filter_corners_topic:=${SCALE_FILTER_CORNERS_TOPIC} _scale_filter_corners_marker_topic:=${SCALE_FILTER_CORNERS_MARKER_TOPIC} _scene_preobserve_enabled:=${SCENE_PREOBSERVE_ENABLED} _scene_preobserve_base_x_offset:=${SCENE_PREOBSERVE_BASE_X_OFFSET} _scene_preobserve_base_y_offset:=${SCENE_PREOBSERVE_BASE_Y_OFFSET} _scene_preobserve_base_z_offset:=${SCENE_PREOBSERVE_BASE_Z_OFFSET} _scene_preobserve_refresh_timeout:=${SCENE_PREOBSERVE_REFRESH_TIMEOUT} _place_bin_color:=${PLACE_BIN_COLOR} _single_cycle:=false _loop_until_no_targets:=true"
}
follow_brain_log() {
  echo "[cleanup] Following /root/.ros/log/robocup_brain.log from cleanup_brain"
  echo "[cleanup] Ctrl+C will stop log following only; containers will keep running."
  docker exec cleanup_brain bash -lc 'while [ ! -f /root/.ros/log/robocup_brain.log ]; do sleep 1; done; tail -F /root/.ros/log/robocup_brain.log'
}

print_summary() {
  cat <<SUMMARY
[cleanup] Hybrid cleanup runtime is up.
[cleanup] ROS_MASTER_URI=$ROS_MASTER_URI
[cleanup] PLACE_BIN_COLOR=$PLACE_BIN_COLOR
[cleanup] brain policy:
  - cube -> direct YOLO grasp pose
  - non-cube -> direct YOLO grasp pose
  - alternate cube and non-cube when both are selectable
  - fall back to the other category when the preferred one has no selectable target
  - grasp_pose_source: ${GRASP_POSE_SOURCE}
  - scene-wide non-cube ranking enabled: ${SCENE_WIDE_NON_CUBE_RANKING}
  - debug_non_cube_only: ${DEBUG_NON_CUBE_ONLY}
  - execute_after_scene_ranking: ${EXECUTE_AFTER_SCENE_RANKING}
  - scene_rank_min_pose_z: ${SCENE_RANK_MIN_POSE_Z}
  - scene_rank_min_top_score: ${SCENE_RANK_MIN_TOP_SCORE}
  - non_cube_bbox_width_margin: ${NON_CUBE_BBOX_WIDTH_MARGIN}
  - cube_place_joints: ${CUBE_PLACE_JOINTS}
  - exclude_cubes_on_scale: ${EXCLUDE_CUBES_ON_SCALE}
  - scale_filter_world_frame: ${SCALE_FILTER_WORLD_FRAME}
  - scale_filter_corner_output_frame: ${SCALE_FILTER_CORNER_OUTPUT_FRAME}
  - scale_filter_corner_world_z: ${SCALE_FILTER_CORNER_WORLD_Z}
  - scale_filter_volume_down_m: ${SCALE_FILTER_VOLUME_DOWN_M}
  - scale_filter_min_world_z: ${SCALE_FILTER_MIN_WORLD_Z}
  - scale_filter_polygon_world_xy: ${SCALE_FILTER_POLYGON_WORLD_XY}
  - publish_scale_filter_corners: ${PUBLISH_SCALE_FILTER_CORNERS}
  - scale_filter_corners_topic: ${SCALE_FILTER_CORNERS_TOPIC}
  - scale_filter_corners_marker_topic: ${SCALE_FILTER_CORNERS_MARKER_TOPIC}
  - scene_preobserve_enabled: ${SCENE_PREOBSERVE_ENABLED}
  - scene_preobserve_base_offset: (${SCENE_PREOBSERVE_BASE_X_OFFSET}, ${SCENE_PREOBSERVE_BASE_Y_OFFSET}, ${SCENE_PREOBSERVE_BASE_Z_OFFSET})
  - scene_preobserve_refresh_timeout: ${SCENE_PREOBSERVE_REFRESH_TIMEOUT}

[cleanup] Containers:
  - perception_grasp
  - cleanup_yolo_cloud
  - cleanup_yolo_json
  - cleanup_motion_control
  - cleanup_brain
SUMMARY
}

if ! wait_for_master; then
  echo "[cleanup] ROS master is not reachable at 127.0.0.1:11311. Start arm_gazebo first." >&2
  exit 1
fi

if ! wait_for_joint_states; then
  echo "[cleanup] /joint_states is not available yet. Make sure arm_gazebo is fully up before running this script." >&2
  exit 1
fi

cleanup_old_runtime

if [ "$START_PATH_PLANNING" -eq 1 ]; then
  start_path_planning
fi
if [ "$START_GRASP_SERVICE" = "true" ]; then
  start_grasp
fi
start_yolo_cloud
start_yolo_json
start_motion_control

# Give YOLO publishers a short warm-up window. The brain will keep waiting
# for detections/point clouds if they are not ready yet.
sleep 5

prepare_brain_params
start_brain
print_summary

if [ "$FOLLOW_LOGS" -eq 1 ]; then
  follow_brain_log
fi
