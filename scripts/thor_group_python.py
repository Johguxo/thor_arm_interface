# Python 2/3 compatibility imports
from __future__ import print_function
from six.moves import input
from decimal import Decimal

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import numpy as np

try:
  from math import pi, tau, dist, fabs, cos
except: # For Python 2 compatibility
  from math import pi, fabs, cos, sqrt
  tau = 2.0*pi
  def dist(p, q):
    return sqrt(sum((p_i - q_i)**2.0 for p_i, q_i in zip(p,q)))
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

def all_close(goal, actual, tolerance):
  """
  Convenience method for testing if the values in two lists are within a tolerance of each other.
  For Pose and PoseStamped inputs, the angle between the two quaternions is compared (the angle 
  between the identical orientations q and -q is calculated correctly).
  @param: goal       A list of floats, a Pose or a PoseStamped
  @param: actual     A list of floats, a Pose or a PoseStamped
  @param: tolerance  A float
  @returns: bool
  """
  if type(goal) is list:
    for index in range(len(goal)):
      if abs(actual[index] - goal[index]) > tolerance:
        return False

  elif type(goal) is geometry_msgs.msg.PoseStamped:
    return all_close(goal.pose, actual.pose, tolerance)

  elif type(goal) is geometry_msgs.msg.Pose:
    x0, y0, z0, qx0, qy0, qz0, qw0 = pose_to_list(actual)
    x1, y1, z1, qx1, qy1, qz1, qw1 = pose_to_list(goal)
    # Euclidean distance
    d = dist((x1, y1, z1), (x0, y0, z0))
    # phi = angle between orientations
    cos_phi_half = fabs(qx0*qx1 + qy0*qy1 + qz0*qz1 + qw0*qw1)
    return d <= tolerance and cos_phi_half >= cos(tolerance / 2.0)

  return True

class MoveGroupThorArm(object):
    def __init__(self):
        super(MoveGroupThorArm, self).__init__()
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('thor_move_group_python', anonymous=True)
        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()    
        group = moveit_commander.MoveGroupCommander("thor_arm")
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)
      
        planning_frame = group.get_planning_frame()
        # We can also print the name of the end-effector link for this group:
        eef_link = group.get_end_effector_link()
        # We can get a list of all the groups in the robot:
        group_names = robot.get_group_names()
        # Misc variables
        self.box_name = ''
        self.robot = robot
        self.scene = scene
        self.group = group
        self.display_trajectory = display_trajectory
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names
        

    def show_information(self):
        robot = self.robot
        group = self.group
        print("============ Planning frame: %s" % group.get_planning_frame())
        print("============ End effector link: %s" % group.get_end_effector_link())
        print("============ Available Planning Groups:", robot.get_group_names())
        print("============ Printing robot state")
        print(robot.get_current_state())
        print("")

    def go_to_joint_state(self):
        move_group = self.group
        """"
        group_variable_values = self.group.get_current_joint_values()

        group_variable_values[0] = 0
        group_variable_values[1] = 0
        group_variable_values[3] = -1.5
        group_variable_values[5] = 1.5
        self.group.set_joint_value_target(group_variable_values)

        plan2 = self.group.plan()
        self.group.go(wait=True)"""
        joint_goal = move_group.get_current_joint_values()
        joint_goal[0] = -tau/4 # 1/4 of a turn
        joint_goal[1] = -tau/8
        joint_goal[2] = -tau/8
        joint_goal[3] = 0  
        joint_goal[4] = 0
        joint_goal[5] = 0
        move_group.go(joint_goal, wait=True)
        
        move_group.stop()
        # For testing:
        current_joints = move_group.get_current_joint_values()
        print("============ Printing robot state")
        print(self.robot.get_current_state())
        print("")
        if all_close(joint_goal, current_joints, 0.01):
            print("other code")
            return True
            #move_group.set_joint_value_target(joint_goal)
            #plan2 = self.group.plan()
            #move_group.go(wait=True)
            #rospy.sleep(5)
    def go_to_pose_goal(self):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        move_group = self.group

        ## BEGIN_SUB_TUTORIAL plan_to_pose
        ##
        ## Planning to a Pose Goal
        ## ^^^^^^^^^^^^^^^^^^^^^^^
        ## We can plan a motion for this group to a desired pose for the
        ## end-effector:
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.orientation.w = 1.0
        pose_goal.position.x = 0
        pose_goal.position.y = -0.25
        pose_goal.position.z = 0.35

        move_group.set_pose_target(pose_goal)

        ## Now, we call the planner to compute the plan and execute it.
        plan = move_group.go(wait=True)
        # Calling `stop()` ensures that there is no residual movement
        move_group.stop()
        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets()
        move_group.clear_pose_targets()
        #Otros intentos
        ## END_SUB_TUTORIAL

        # For testing:
        # Note that since this section of code will not be included in the tutorials
        # we use the class variable rather than the copied state variable
        current_pose = self.group.get_current_pose().pose
        """print("============ Printing current pose")
        print(current_pose)
        print("")
        print("============ Printing robot state")
        print(self.robot.get_current_state())
        print("")"""
        return all_close(pose_goal, current_pose, 0.01)
    def go_to_pose_goal_method_two(self):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        
        move_group = self.group
        current_pose = move_group.get_current_pose().pose
        ## BEGIN_SUB_TUTORIAL plan_to_pose
        ##
        ## Planning to a Pose Goal
        ## ^^^^^^^^^^^^^^^^^^^^^^^
        ## We can plan a motion for this group to a desired pose for the
        ## end-effector:
        pose_goal = current_pose
        pose_goal.position.x -= 0.05

        move_group.set_pose_target(pose_goal)

        ## Now, we call the planner to compute the plan and execute it.
        plan = move_group.go(wait=True)
        # Calling `stop()` ensures that there is no residual movement
        move_group.stop()
        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets()
        move_group.clear_pose_targets()
        #Otros intentos
        ## END_SUB_TUTORIAL

        # For testing:
        # Note that since this section of code will not be included in the tutorials
        # we use the class variable rather than the copied state variable
        current_pose = self.group.get_current_pose().pose
        """print("============ Printing current pose")
        print(current_pose)
        print("")
        print("============ Printing robot state")
        print(self.robot.get_current_state())
        print("")"""
        return all_close(pose_goal, current_pose, 0.01)
    def go_to_new_joint_states(self):
        move_group = self.group
        #while all_close(pose_goal, current_pose, 0.01):
        joint_goal = move_group.get_current_joint_values()

        joint_goal[0] -= 0
        joint_goal[1] -= tau/16
        joint_goal[2] -= tau/16 # 1/4 of a turn
        move_group.go(joint_goal, wait=True)
        move_group.stop()
        current_joints = move_group.get_current_joint_values()
        if all_close(joint_goal, current_joints, 0.01):
            print("No esta cerca")
        joint_goal = move_group.get_current_joint_values()
        joint_goal[0] -= 0
        joint_goal[1] -= tau/16
        move_group.go(joint_goal, wait=True)
        move_group.stop()
        current_joints = move_group.get_current_joint_values()
        if all_close(joint_goal, current_joints, 0.01):
            print("No esta cerca")
            return True
    
    def presentation_ros(self):
        move_group = self.group
        joint_goal = move_group.get_current_joint_values()
        joint_goal[0] = -tau/4-tau/16 # 1/4 of a turn
        joint_goal[1] = -3*tau/16
        joint_goal[2] = -tau/8 - tau/16
        joint_goal[3] = 0 
        joint_goal[4] = 0
        joint_goal[5] = 0
        move_group.go(joint_goal, wait=True)
        move_group.stop()

        """"current_pose = move_group.get_current_pose().pose
        pose_goal = current_pose
        pose_goal.position.y -= 0.1
        pose_goal.position.z += 0.001
        move_group.set_pose_target(pose_goal)
        plan = move_group.go(wait=True)
        move_group.clear_pose_targets()

        current_pose = move_group.get_current_pose().pose
        pose_goal = current_pose
        pose_goal.position.x += 0.15
        pose_goal.position.y -= 0.15
        move_group.set_pose_target(pose_goal)
        plan = move_group.go(wait=True)
        move_group.clear_pose_targets()"""

        current_pose = move_group.get_current_pose().pose
        pose_goal = current_pose
        pose_goal.position.x += 0.15
        pose_goal.position.z -= 0.001
        move_group.set_pose_target(pose_goal)
        plan = move_group.go(wait=True)
        move_group.clear_pose_targets()

        current_pose = move_group.get_current_pose().pose
        pose_goal = current_pose
        pose_goal.position.y += 0
        pose_goal.position.x += 0.05
        move_group.set_pose_target(pose_goal)
        plan = move_group.go(wait=True)
        move_group.clear_pose_targets()

        joint_goal = move_group.get_current_joint_values()
        joint_goal[0] = joint_goal[0] # 1/4 of a turn
        joint_goal[1] = joint_goal[1]
        joint_goal[2] = joint_goal[2]
        joint_goal[3] = joint_goal[3]
        joint_goal[4] = joint_goal[4]-tau/16
        joint_goal[5] = joint_goal[5]-tau/16
        move_group.go(joint_goal, wait=True)
        move_group.clear_pose_targets()

        joint_goal = move_group.get_current_joint_values()
        joint_goal[0] = joint_goal[0]+tau/32 # 1/4 of a turn
        joint_goal[1] = joint_goal[1]
        joint_goal[2] = joint_goal[2]
        joint_goal[3] = joint_goal[3]
        joint_goal[4] = joint_goal[4]
        joint_goal[5] = joint_goal[5]
        move_group.go(joint_goal, wait=True)
        move_group.clear_pose_targets()

        current_pose = move_group.get_current_pose().pose
        pose_goal = current_pose
        pose_goal.position.y += -1
        pose_goal.position.x += -1
        move_group.set_pose_target(pose_goal)
        plan = move_group.go(wait=True)
        move_group.clear_pose_targets()
        move_group.stop()
    def move_initial_pose(self):
        move_group = self.group
        current_pose = move_group.get_current_pose().pose
        add_x = 0.28
        add_y = -0.2
        add_z = -0.3
        pose_goal = current_pose
        pose_goal.position.x += add_x
        pose_goal.position.y += add_y
        pose_goal.position.z += add_z
        move_group.set_pose_target(pose_goal)
        #It will be automatically displayed in rviz.
        plan = move_group.plan()
        plan = move_group.go(wait=True)
        print("go",plan)
        move_group.clear_pose_targets()

    def move_respect_yz(self):
        move_group = self.group
        current_pose = move_group.get_current_pose().pose
        add_x = -(current_pose.position.x*2)
        pose_goal = current_pose
        pose_goal.position.x += add_x
        pose_goal.position.y += 0
        pose_goal.position.z += 0
        print("Este objetivo")
        print(pose_goal.position)
        move_group.set_pose_target(pose_goal)
        plan = move_group.go(wait=True)
        move_group.clear_pose_targets()

    def move_alleatory(self):
        move_group = self.group
        current_pose = move_group.get_current_pose().pose
        flag = False
        error_radius = 0.005
        error_z = 0.005
        radius = 0
        rand_x = 0
        rand_z = 0
        rand_y = 0
        add_x = -(current_pose.position.x*2)
        print("-- Mi pose inicial --")
        print(current_pose)
        while not flag:
          rand_z += error_z
          while radius <= 1:
            radius += error_radius
            print("Radio: ", radius,"Aumento en z: ",rand_z)
            for th in np.arange(0, 6.28, 0.02):
              rand_x = radius*np.cos(th)
              rand_y = radius*np.sin(th)
              pose_goal = current_pose
              pose_goal.position.x += add_x + rand_x
              pose_goal.position.y += rand_y
              pose_goal.position.z += rand_z
              move_group.set_pose_target(pose_goal)
              plan = move_group.go(wait=True)
              #print("Go: ",plan,type(plan))
              move_group.clear_pose_targets()
              flag = plan
              if flag:
                break
        print("-- Este objetivo --")
        print(pose_goal)
        #time_seconds = move_group.set_planning_time(self)
        plan = move_group.plan()
        plan = move_group.go(wait=True)
        move_group.clear_pose_targets()
    
    def move_respect_y_up_z(self):
        move_group = self.group
        current_pose = move_group.get_current_pose().pose
        add_x = -(current_pose.position.x*2) + 0.005
        add_z = (current_pose.position.z)/4
        pose_goal = current_pose
        pose_goal.position.x += add_x
        pose_goal.position.y += 0
        pose_goal.position.z += add_z
        print("Objetivo")
        print(pose_goal)
        move_group.set_pose_target(pose_goal)
        plan = move_group.go(wait=True)
        move_group.clear_pose_targets()
    
    def move_respect_y_down_z(self):
        move_group = self.group
        current_pose = move_group.get_current_pose().pose
        add_x = -(current_pose.position.x*2) - 0.005
        add_z = -0.02
        pose_goal = current_pose
        pose_goal.position.x += add_x
        pose_goal.position.y += 0
        pose_goal.position.z += add_z
        
        print("Objetivo")
        print(pose_goal)
        move_group.set_pose_target(pose_goal)
        plan = move_group.go(wait=True)
        move_group.clear_pose_targets()

    def move_add_pos(self,addpos):
        move_group = self.group
        current_pose = move_group.get_current_pose().pose
        pose_goal = current_pose
        pose_goal.position.x += addpos["add_x"]
        pose_goal.position.y += addpos["add_y"]
        pose_goal.position.z += addpos["add_z"]
        move_group.set_pose_target(pose_goal)
        plan = move_group.go(wait=True)
        move_group.clear_pose_targets()

    def show_current_pos_and_ori(self):
        move_group = self.group
        current_pose = move_group.get_current_pose().pose
        print(current_pose)
def main():
    try:

        print("")
        print("----------------------------------------------------------")
        print("Movimiento de Brazo robotico de 6GDL")
        print("----------------------------------------------------------")
        print("Presionar Ctrl-D para salir en cualquier momento")
        print("")
        input("============ Presionar `Enter` para comenzar el moveit_commander ...")
        tutorial = MoveGroupThorArm()
        while True:
          print("----------------------------------------------------------")
          print("Escoger una opcion: ")
          print("1)Informacion del robot")
          print("2)Ver pos y orien. en este momento")
          print("3)Incrementar valores a las pocisiones")
          print("4)Pocision inicial")
          print("5)Movimiento espejo con respecto a Y-Z")
          print("6)Movimiento espejo con respecto a Y-Z con un aumento en Z")
          print("7)Movimiento espejo con respecto a Y-Z con una disminucion en Z")
          print("8)Movimiento aleatorio")
          print("Presionar Ctrl-D para salir en cualquier momento")
          s = input("---> ")
          if(s == "1"):
            tutorial.show_information()
          if(s == "2"):
            tutorial.show_current_pos_and_ori()
          elif(s == "3"):
            _addx = float(input("+x -> "))
            _addy = float(input("+y -> "))
            _addz = float(input("+z -> "))
            add_pos = {"add_x":_addx,
                      "add_y":_addy,
                      "add_z":_addz}
            tutorial.move_add_pos(add_pos)
          elif(s == "4"):
            tutorial.move_initial_pose()
          elif(s == "5"):
            tutorial.move_respect_yz()
          elif(s == "6"):
            tutorial.move_respect_y_up_z()
          elif(s == "7"):
            tutorial.move_respect_y_down_z()
          elif(s == "8"):
            tutorial.move_alleatory()
        #input("============ Press `Enter` to execute a movement using a joint state goal ...")
        #tutorial.go_to_joint_state()
        #input("============ Press `Enter` to execute a movement using a pose goal ...")
        #tutorial.go_to_pose_goal()
        #input("============ Press `Enter` to execute a movement using a pose goal other method ...")
        #tutorial.go_to_pose_goal_method_two()
        #input("============ Press `Enter` to execute a movement using a diferents joint state goal ...")
        #tutorial.go_to_new_joint_states()
        input("============ Press `Enter` to execute a movement ...")
        tutorial.presentation_ros()
        input("============ Press `Enter` to execute a movement ...")
        moveit_commander.roscpp_shutdown()
    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return

if __name__ == '__main__':
  main()