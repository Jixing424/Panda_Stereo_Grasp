#!/usr/bin/env python3
import rospy
from my_arm_controller.msg import QRCode
from moveit_commander import MoveGroupCommander
from geometry_msgs.msg import Pose
import actionlib
from franka_gripper.msg import GraspAction, GraspGoal, MoveAction, MoveGoal
import franka_gripper




class PandaCatch():
    def __init__(self):
        self.sub = rospy.Subscriber("/qr_pose", QRCode, self.qr_pose_callback)
        self.arm = MoveGroupCommander("panda_manipulator")
        self.hand = MoveGroupCommander('panda_hand')
        
        self.qr_info_dict = {}
        self.x_error = -0.15
        self.y_error = 0.0
        self.z_error = 0.05
        
        self.grasp_client = actionlib.SimpleActionClient(
            '/franka_gripper/grasp', GraspAction)
        self.move_client = actionlib.SimpleActionClient(
            '/franka_gripper/move', MoveAction)
        self.grasp_client.wait_for_server()
        self.move_client.wait_for_server()
    
    def qr_pose_callback(self, msg):
        self.qr_info_dict[msg.id] = (msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)
        
    def move_to_box(self, box_id):
        success = False
        print('开始抓取')
        if box_id in self.qr_info_dict.keys():
            
            (x, y, z) = self.qr_info_dict[box_id]
            pose_goal = Pose()
            # 让机械臂运动到指定位置
            pose_goal.position.x = x + self.x_error
            pose_goal.position.y = y + self.y_error
            pose_goal.position.z = z + self.z_error

            pose_goal.orientation.x = 1.0
            pose_goal.orientation.y = 0.0
            pose_goal.orientation.z = 0.0
            pose_goal.orientation.w = 0.0
            
            self.arm.set_pose_target(pose_goal)
            success = self.arm.go(wait=True)
            self.arm.clear_pose_targets()
            if success:
                print('机械臂已运动到指定位置')
            else:
                print('机械臂运动失败')
        return success
    
    def open_gripper(self):
        self.hand.set_named_target('open')
        success = self.hand.go(wait=True)
        if success:
            print('夹爪已打开')
        else:
            print('夹爪打开失败')
        return success

    
    def close_gripper(self, width=0.06, force=30, speed=0.05):
        grasp_goal = GraspGoal()
        grasp_goal.width = width
        grasp_goal.speed = speed
        grasp_goal.force = force
        grasp_goal.epsilon.inner = 0.005  # 内部容差5mm
        grasp_goal.epsilon.outer = 0.005  # 外部容差5mm
        
        self.grasp_client.send_goal(grasp_goal)
        self.grasp_client.wait_for_result(rospy.Duration(5.0))
        
        result = self.grasp_client.get_state()
        if result == actionlib.GoalStatus.SUCCEEDED:
            print(f"抓取成功! 力度: {force}N")
            return True
        else:
            print("抓取失败")
            return False
    
    def pull_down(self):
        pose = self.arm.get_current_pose().pose
        pose.position.z -= 0.05
        self.arm.set_pose_target(pose)
        success = self.arm.go(wait=True)
        self.arm.clear_pose_targets()
        if success:
            print('机械臂下降')
        else:
            print('机械臂下降失败')
        return success
    
    def pull_up(self):
        pose = self.arm.get_current_pose().pose
        pose.position.z += 0.1
        self.arm.set_pose_target(pose)
        success = self.arm.go(wait=True)
        self.arm.clear_pose_targets()
        if success:
            print('机械臂上升')
        else:
            print('机械臂上升失败')
        return success
    
    def arm_to_home(self):
        self.arm.set_named_target('ready')
        success = self.arm.go(wait=True)
        if success:
            print('机械臂回到起始点')
        else:
            print('机械臂回到起始点失败')
        return success
            
    def catch(self, box_id):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if box_id in self.qr_info_dict:
                print('已监听到指定的id消息')
                break
            rate.sleep()
            
        self.move_to_box(box_id)
        self.open_gripper()
        self.pull_down()
        self.close_gripper()
        self.pull_up()
        self.arm_to_home()
        self.open_gripper()
        

if __name__ == "__main__":
    rospy.init_node("arm_control")
    pandacatch = PandaCatch()
    
    pandacatch.catch('ID=001')
    pandacatch.catch('ID=002')
    pandacatch.catch('ID=003')
    
    
    