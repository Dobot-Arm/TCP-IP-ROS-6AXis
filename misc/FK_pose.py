#!/usr/bin/env python3
import rospy
import math
import threading
import time
from datetime import datetime

from sensor_msgs.msg import JointState
from moveit_msgs.srv import GetPositionFK, GetPositionFKRequest
from moveit_msgs.msg import RobotState
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header


class ContinuousFKCalculator:
    def __init__(self, frequency=1.0):
        rospy.init_node('continuous_fk_calculator', anonymous=True)

        # CR3 关节名（与你实际一致）
        self.joint_names = [
            'joint1', 'joint2', 'joint3',
            'joint4', 'joint5', 'joint6'
        ]

        # 订阅关节状态
        self.joint_state_sub = rospy.Subscriber(
            '/joint_states',
            JointState,
            self.joint_state_callback,
            queue_size=10
        )

        # FK 服务
        rospy.loginfo("等待 /compute_fk 服务...")
        rospy.wait_for_service('/compute_fk')
        self.fk_client = rospy.ServiceProxy(
            '/compute_fk',
            GetPositionFK
        )
        rospy.loginfo("✅ FK 服务已连接")

        # 状态
        self.current_joint_state = None
        self.lock = threading.Lock()
        self.joint_data_available = threading.Event()

        # 控制参数
        self.frequency = frequency
        self.interval = 1.0 / frequency

        # 统计
        self.count = 0
        self.success = 0
        self.fail = 0

        self.running = True

        # 启动线程
        threading.Thread(target=self.calculation_loop, daemon=True).start()

        rospy.loginfo("\n" + "="*70)
        rospy.loginfo("Dobot CR3 连续 FK 计算器（ROS 1）")
        rospy.loginfo(f"频率: {frequency} Hz")
        rospy.loginfo("="*70)

    # ------------------- 回调 -------------------
    def joint_state_callback(self, msg):
        with self.lock:
            try:
                angles = []
                for name in self.joint_names:
                    angles.append(msg.position[msg.name.index(name)])
                self.current_joint_state = angles
                self.joint_data_available.set()
            except ValueError:
                pass

    # ------------------- 主循环 -------------------
    def calculation_loop(self):
        while not rospy.is_shutdown() and self.running:
            if not self.joint_data_available.wait(timeout=1.0):
                continue

            self.count += 1
            start = time.time()

            with self.lock:
                joints = self.current_joint_state[:] if self.current_joint_state else None

            if joints is None:
                continue

            pose = self.compute_fk(joints)
            if pose:
                self.success += 1
                self.print_result(pose, joints, start)
            else:
                self.fail += 1

            if self.count % 10 == 0:
                self.print_stats()

            dt = self.interval - (time.time() - start)
            if dt > 0:
                time.sleep(dt)

    # ------------------- FK 计算 -------------------
    def compute_fk(self, joints):
        try:
            req = GetPositionFKRequest()
            req.header = Header(
                frame_id='base_link',
                stamp=rospy.Time.now()
            )
            req.fk_link_names = ['Link6']

            js = JointState()
            js.name = self.joint_names
            js.position = joints

            rs = RobotState()
            rs.joint_state = js
            rs.is_diff = False

            req.robot_state = rs

            resp = self.fk_client(req)

            if resp.error_code.val == 1 and resp.pose_stamped:
                return resp.pose_stamped[0].pose

        except rospy.ServiceException as e:
            rospy.logerr(f"FK 服务异常: {e}")

        return None

    # ------------------- 输出 -------------------
    def print_result(self, pose, joints, start):
        t = datetime.now().strftime("%H:%M:%S")
        rospy.loginfo(f"\n[{t}] 计算 #{self.count} "
                      f"({(time.time()-start)*1000:.1f} ms)")
        rospy.loginfo("-" * 60)

        for n, a in zip(self.joint_names, joints):
            rospy.loginfo(f"{n}: {a:.6f} ({math.degrees(a):.2f}°)")

        p = pose.position
        rospy.loginfo(f"\n位置: x={p.x:.6f}  y={p.y:.6f}  z={p.z:.6f}")

        r, pi, y = self.quat_to_euler(pose.orientation)
        rospy.loginfo(f"姿态: R={math.degrees(r):.2f}° "
                      f"P={math.degrees(pi):.2f}° "
                      f"Y={math.degrees(y):.2f}°")

    def quat_to_euler(self, q):
        x, y, z, w = q.x, q.y, q.z, q.w
        sinr = 2*(w*x + y*z)
        cosr = 1 - 2*(x*x + y*y)
        roll = math.atan2(sinr, cosr)

        sinp = 2*(w*y - z*x)
        pitch = math.asin(sinp) if abs(sinp) < 1 else math.copysign(math.pi/2, sinp)

        siny = 2*(w*z + x*y)
        cosy = 1 - 2*(y*y + z*z)
        yaw = math.atan2(siny, cosy)

        return roll, pitch, yaw

    def print_stats(self):
        rospy.loginfo("\n📊 统计")
        rospy.loginfo(f"总: {self.count}  成功: {self.success}  失败: {self.fail}")

    def shutdown(self):
        self.running = False


if __name__ == '__main__':
    try:
        node = ContinuousFKCalculator(frequency=1.0)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
