#!/usr/bin/env python3
"""
GraspNet Estimator Node
Author: Muye Yuan
Environment: CUDA 11.3 + ROS Noetic (GraspNet-1Billion legacy requirements)
"""

import rospy
import numpy as np
import torch
from cv_bridge import CvBridge, CvBridgeError

from sensor_msgs.msg import Image, PointCloud2, CameraInfo
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from common_msgs.msg import GraspCandidate, DetectedObject
import sensor_msgs.point_cloud2 as pc2

try:
    import open3d as o3d
except ImportError:
    rospy.logwarn("[Grasp] open3d not installed. Install with: pip install open3d")


class GraspEstimatorNode:
    def __init__(self):
        rospy.init_node('grasp_estimator', anonymous=False)
        rospy.loginfo("=" * 60)
        rospy.loginfo("GraspNet Estimator Node Initializing")
        rospy.loginfo("=" * 60)
        
        # 设备检查（CUDA 11.3）
        self.device = self._setup_device()
        
        # 参数配置
        self.checkpoint_path = rospy.get_param('~checkpoint_path', 'graspnet_checkpoints/checkpoint.tar')
        self.num_grasp_candidates = rospy.get_param('~num_grasp_candidates', 5)
        self.pointcloud_topic = rospy.get_param('~pointcloud_topic', '/camera/depth/points')
        
        # 加载 GraspNet 模型
        self.model = self._load_graspnet_model()
        
        # ROS 接口
        self.bridge = CvBridge()
        
        # 订阅点云数据
        self.pointcloud_sub = rospy.Subscriber(
            self.pointcloud_topic,
            PointCloud2,
            self.pointcloud_callback,
            queue_size=1,
            buff_size=2**24
        )
        
        # 订阅检测结果（可选：根据检测区域裁剪点云）
        self.detection_sub = rospy.Subscriber(
            '/perception/detected_objects',
            DetectedObject,
            self.detection_callback,
            queue_size=10
        )
        
        # 发布抓取候选
        self.grasp_pub = rospy.Publisher(
            '/perception/grasp_candidates',
            GraspCandidate,
            queue_size=10
        )
        
        self.detected_objects = []
        
        rospy.loginfo(f"[Grasp] Device: {self.device}")
        rospy.loginfo(f"[Grasp] Checkpoint: {self.checkpoint_path}")
        rospy.loginfo(f"[Grasp] Subscribing to: {self.pointcloud_topic}")
        rospy.loginfo("[Grasp] Initialization complete. Ready to estimate grasps!")
        
    def _setup_device(self):
        """设置 CUDA 11.3 环境"""
        if torch.cuda.is_available():
            device = torch.device('cuda')
            cuda_version = torch.version.cuda
            rospy.loginfo(f"[Grasp] CUDA available: {torch.cuda.get_device_name(0)}")
            rospy.loginfo(f"[Grasp] CUDA version: {cuda_version}")
            
            # 警告：如果 CUDA 版本不是 11.3
            if cuda_version != "11.3":
                rospy.logwarn(f"[Grasp] Expected CUDA 11.3, but got {cuda_version}. GraspNet may have compatibility issues.")
        else:
            device = torch.device('cpu')
            rospy.logwarn("[Grasp] CUDA not available. Running on CPU (NOT RECOMMENDED for GraspNet)")
            
        return device
        
    def _load_graspnet_model(self):
        """加载 GraspNet-1Billion 模型"""
        # TODO: 实际集成 GraspNet-1Billion
        # 参考: https://github.com/graspnet/graspnet-baseline
        rospy.loginfo("[Grasp] Loading GraspNet model (placeholder)...")
        
        # 伪代码示例（需替换为真实 GraspNet 加载逻辑）:
        # from graspnetAPI import GraspNet
        # model = GraspNet(checkpoint_path=self.checkpoint_path)
        # model.to(self.device)
        # model.eval()
        
        rospy.logwarn("[Grasp] GraspNet model loading is a PLACEHOLDER. Integrate graspnet-baseline here.")
        return None
        
    def detection_callback(self, msg):
        """接收检测结果，用于裁剪点云"""
        self.detected_objects.append(msg)
        rospy.logdebug(f"[Grasp] Received detection: {msg.label}")
        
    def pointcloud_callback(self, msg):
        """处理点云并估计抓取姿态"""
        rospy.loginfo("[Grasp] Processing point cloud...")
        
        try:
            # ROS PointCloud2 -> NumPy array
            points = self._ros_pointcloud_to_numpy(msg)
            
            if points is None or len(points) == 0:
                rospy.logwarn("[Grasp] Empty point cloud received")
                return
                
            # 预处理点云（下采样、滤波等）
            processed_points = self._preprocess_pointcloud(points)
            
            # 执行抓取推理
            grasp_poses = self._estimate_grasps(processed_points)
            
            # 发布抓取候选
            for i, (pose, quality) in enumerate(grasp_poses[:self.num_grasp_candidates]):
                self._publish_grasp_candidate(pose, quality, msg.header.frame_id)
                rospy.loginfo(f"[Grasp] Candidate {i+1}: quality={quality:.3f}")
                
        except Exception as e:
            rospy.logerr(f"[Grasp] Error processing point cloud: {e}")
            
    def _ros_pointcloud_to_numpy(self, msg):
        """将 ROS PointCloud2 转换为 NumPy 数组"""
        points_list = []
        
        for point in pc2.read_points(msg, skip_nans=True, field_names=("x", "y", "z")):
            points_list.append([point[0], point[1], point[2]])
            
        return np.array(points_list, dtype=np.float32)
        
    def _preprocess_pointcloud(self, points):
        """预处理点云（下采样、去噪等）"""
        if len(points) == 0:
            return points
            
        # 使用 Open3D 进行预处理
        try:
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(points)
            
            # 下采样
            pcd = pcd.voxel_down_sample(voxel_size=0.005)
            
            # 统计滤波去除离群点
            pcd, _ = pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
            
            return np.asarray(pcd.points)
        except Exception as e:
            rospy.logwarn(f"[Grasp] Preprocessing failed: {e}. Using raw points.")
            return points
            
    def _estimate_grasps(self, points):
        """
        使用 GraspNet 估计抓取姿态
        返回: [(pose, quality), ...]
        """
        # TODO: 实际 GraspNet 推理
        # 伪代码:
        # grasp_group = self.model.predict(points)
        # return [(grasp.pose, grasp.score) for grasp in grasp_group]
        
        # 占位符：返回随机抓取姿态
        rospy.logwarn("[Grasp] Using PLACEHOLDER grasp estimation!")
        
        dummy_grasps = []
        for i in range(self.num_grasp_candidates):
            # 随机位置（在点云中心附近）
            center = np.mean(points, axis=0)
            position = center + np.random.randn(3) * 0.05
            
            # 随机姿态（四元数）
            orientation = [0.0, 0.0, 0.0, 1.0]  # [x, y, z, w]
            
            pose = Pose()
            pose.position = Point(x=position[0], y=position[1], z=position[2])
            pose.orientation = Quaternion(x=orientation[0], y=orientation[1], 
                                         z=orientation[2], w=orientation[3])
            
            quality = np.random.uniform(0.5, 1.0)
            
            dummy_grasps.append((pose, quality))
            
        return dummy_grasps
        
    def _publish_grasp_candidate(self, pose, quality, frame_id):
        """发布单个抓取候选"""
        msg = GraspCandidate()
        
        msg.pose = PoseStamped()
        msg.pose.header.stamp = rospy.Time.now()
        msg.pose.header.frame_id = frame_id
        msg.pose.pose = pose
        
        msg.quality = quality
        
        self.grasp_pub.publish(msg)
        
    def run(self):
        """保持节点运行"""
        rospy.spin()


if __name__ == '__main__':
    try:
        node = GraspEstimatorNode()
        node.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("[Grasp] Shutting down")
