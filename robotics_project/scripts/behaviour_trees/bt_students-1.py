#!/usr/bin/env python

import py_trees as pt, py_trees_ros as ptr, rospy
from reactive_sequence import RSequence
from std_srvs.srv import Empty, SetBool, SetBoolRequest 

from geometry_msgs.msg import Twist, PoseStamped
from actionlib import SimpleActionClient
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal
from robotics_project.srv import MoveHead, MoveHeadRequest, MoveHeadResponse

flag = 0

class BehaviourTree(ptr.trees.BehaviourTree):

	def __init__(self):
		global flag
		flag=0

		rospy.loginfo("Initialising behaviour tree")

		# tuck the arm
		b01 = tuckarm()

		# lower head
		b02 = movehead("down")

		b0 = RSequence(
			name="Detection",
			children=[b01, b02]
		)

		b10 = notDetect()

		# call pick_srv
		b2 = picking()

		# rotate
		b31 = pt.composites.Selector(
			name="Rotate fallback",
			children=[counter(60, "At table?"), go("Rotate!", 0, -0.5)]
		)

		# move forward
		b32 = pt.composites.Selector(
			name="Move forward fallback",
			children=[counter(18, "Rotated?"), go("Go to table!", 0.5, 0)]
		)

		# move to table 2
		b3 = RSequence(
			name="Go to table sequence",
			children=[b31, b32]
		)

		# call place_srv
		b4 = placing()

		# Check cube detection
		b5 = detect()

		# move arm to safe position
		b60 = safePosition()

		# rotate
		b61 = pt.composites.Selector(
			name="Rotate fallback",
			children=[counter(60, "At table?"), go("Rotate!", 0, 0.5)]
		)

		# move forward
		b62 = pt.composites.Selector(
			name="Move forward fallback",
			children=[counter(18, "Rotated?"), go("Go to table!", 0.5, 0)]
		)

		b63=restart()

		# move to table 1
		b6 = RSequence(
			name="Back to table sequence",
			children=[b60, b61, b62, b63]
		)

		
		b7 = pt.composites.Selector(
			name="Finish task?",
			children=[b5, b6]
		)

		b11 = RSequence(
			name="Complete pick and place sequence",
			children=[b2, b3, b4, b7]
		)

		b1 = pt.composites.Selector(
			name="Detecting fallback",
			children=[b10, b11]
		)

		# become the treefrom geometry_msgs.msg import PoseStamped
		tree = RSequence(name="Main sequence", children=[b0, b1])
		super(BehaviourTree, self).__init__(tree)
		

		# execute the behaviour tree
		rospy.sleep(5)
		self.setup(timeout=10000)
		while not rospy.is_shutdown(): 
			self.tick_tock(1)



class counter(pt.behaviour.Behaviour):

	"""
	Returns running for n ticks and success thereafter.
	"""

	def __init__(self, n, name):

		rospy.loginfo("Initialising counter behaviour.")

		# counter
		self.i = 0
		self.n = n

		# become a behaviour
		super(counter, self).__init__(name)

	def update(self):
		global flag

		if flag==0:

			# increment i
			self.i += 1

			# succeed after count is done
			return pt.common.Status.FAILURE if self.i <= self.n else pt.common.Status.SUCCESS
		else:
			self.i=0
			return pt.common.Status.SUCCESS



class go(pt.behaviour.Behaviour):

    """
    Returns running and commands a velocity indefinitely.
    """

    def __init__(self, name, linear, angular):

        rospy.loginfo("Initialising go behaviour.")

        # action space
        #self.cmd_vel_top = rospy.get_param(rospy.get_name() + '/cmd_vel_topic')
        self.cmd_vel_top = "/key_vel"
        #rospy.loginfo(self.cmd_vel_top)
        self.cmd_vel_pub = rospy.Publisher(self.cmd_vel_top, Twist, queue_size=10)
        
        # command
        self.move_msg = Twist()
        self.move_msg.linear.x = linear
        self.move_msg.angular.z = angular

        # become a behaviour
        super(go, self).__init__(name)

    def update(self):

        # send the message
        rate = rospy.Rate(10)
        self.cmd_vel_pub.publish(self.move_msg)
        rate.sleep()

        # tell the tree that you're running
        return pt.common.Status.RUNNING

class tuckarm(pt.behaviour.Behaviour):

	"""
	Sends a goal to the tuck arm action server.
	Returns running whilst awaiting the result,
	success if the action was succesful, and v.v..
	"""

	def __init__(self):

		rospy.loginfo("Initialising tuck arm behaviour.")

		# Set up action client
		self.play_motion_ac = SimpleActionClient("/play_motion", PlayMotionAction)

		# personal goal setting
		self.goal = PlayMotionGoal()
		self.goal.motion_name = 'home'
		self.goal.skip_planning = True

		# execution checker
		self.sent_goal = False
		self.finished = False

		# become a behaviour
		super(tuckarm, self).__init__("Tuck arm!")

	def update(self):
		global flag

		if flag==0:
            
			# already tucked the arm
			if self.finished: 
				return pt.common.Status.SUCCESS
			
			# command to tuck arm if haven't already
			elif not self.sent_goal:

				# send the goal
				self.play_motion_ac.send_goal(self.goal)
				self.sent_goal = True

				# tell the tree you're running
				return pt.common.Status.RUNNING

			# if I was succesful! :)))))))))
			elif self.play_motion_ac.get_result():

				# than I'm finished!
				self.finished = True
				return pt.common.Status.SUCCESS

			# if failed
			elif not self.play_motion_ac.get_result():
				return pt.common.Status.FAILURE

			# if I'm still trying :|
			else:
				return pt.common.Status.RUNNING
		else:
			self.sent_goal = False
			self.finished = False
			return pt.common.Status.SUCCESS



class movehead(pt.behaviour.Behaviour):

	"""
	Lowers or raisesthe head of the robot.
	Returns running whilst awaiting the result,
	success if the action was succesful, and v.v..
	"""

	def __init__(self, direction):

		rospy.loginfo("Initialising move head behaviour.")

		# server
		mv_head_srv_nm = rospy.get_param(rospy.get_name() + '/move_head_srv')
		self.move_head_srv = rospy.ServiceProxy(mv_head_srv_nm, MoveHead)
		rospy.wait_for_service(mv_head_srv_nm, timeout=30)

		# head movement direction; "down" or "up"
		self.direction = direction

		# execution checker
		self.tried = False
		self.done = False

		# become a behaviour
		super(movehead, self).__init__("Lower head!")

	def update(self):
		global flag
		if flag==0:

			# success if done
			if self.done:
				return pt.common.Status.SUCCESS

			# try if not tried
			elif not self.tried:

				# command
				self.move_head_req = self.move_head_srv(self.direction)
				self.tried = True

				# tell the tree you're running
				return pt.common.Status.RUNNING

			# if succesful
			elif self.move_head_req.success:
				self.done = True
				return pt.common.Status.SUCCESS

			# if failed
			elif not self.move_head_req.success:
				return pt.common.Status.FAILURE

			# if still trying
			else:
				return pt.common.Status.RUNNING
		else:
			self.tried = False
			self.done = False
			return pt.common.Status.SUCCESS




class safePosition(pt.behaviour.Behaviour):

	"""
	Moves arm to safe position.
	Returns running whilst awaiting the result,
	success if the action was succesful, and v.v..
	"""

	def __init__(self):

		rospy.loginfo("Initialising safe position behaviour.")

		# Set up action client
		self.play_motion_ac = SimpleActionClient("/play_motion", PlayMotionAction)

		# personal goal setting
		self.goal = PlayMotionGoal()
		self.goal.motion_name = 'pick_final_pose'
		self.goal.skip_planning = True

		# execution checker
		self.sent_goal = False
		self.finished = False

		# become a behaviour
		super(safePosition, self).__init__("Safe position!")

	def update(self):
		global flag

		if flag==0:

			# already tucked the arm
			if self.finished: 
				return pt.common.Status.SUCCESS

			# command to tuck arm if haven't already
			elif not self.sent_goal:

				# send the goal
				self.play_motion_ac.send_goal(self.goal)
				self.sent_goal = True

				# tell the tree you're running
				return pt.common.Status.RUNNING

			# if I was succesful! :)))))))))
			elif self.play_motion_ac.get_result():

				# than I'm finished!
				self.finished = True
				return pt.common.Status.SUCCESS

			# if failed
			elif not self.play_motion_ac.get_result():
				return pt.common.Status.FAILURE

			# if I'm still trying :|
			else:
				return pt.common.Status.RUNNING
		
		else:
			self.sent_goal = False
			self.finished = False
			return pt.common.Status.SUCCESS



		

class picking(pt.behaviour.Behaviour):

	"""
	Connects to pick_srv to send the goal pose.
	Performs the picking task.
	Returns running whilst awaiting the result,
	success if the action was succesful, and v.v..
	"""

	def __init__(self):

		rospy.loginfo("Initialising picking behaviour.")

		# server
		self.pick_srv = rospy.get_param(rospy.get_name() + '/pick_srv')

		rospy.loginfo("Connecting to pick service")
		self.scene_srv = rospy.ServiceProxy(self.pick_srv, SetBool)
		rospy.wait_for_service(self.pick_srv, timeout=30)
		rospy.loginfo("Connected")

		# execution checker
		self.service_called = False
		self.finished = False
		self.result = SetBoolRequest()

		# become a behaviour
		super(picking, self).__init__("Picking task")

	def update(self):
		global flag


		if flag==0:
			# already pick the cube
			if self.finished: 
				return pt.common.Status.SUCCESS

			# command to pick the cube if it is not picked 
			elif not self.service_called:

				# call the service
				self.result = self.scene_srv(False)
				self.service_called = True

				# tell the tree you're running
				return pt.common.Status.RUNNING

			# if I was succesful! :)))))))))
			elif self.result.success:

				# then I'm finished!
				self.finished = True
				return pt.common.Status.SUCCESS

			# if failed
			elif not self.result.success:
				return pt.common.Status.FAILURE

			# if I'm still trying :|
			else:
				return pt.common.Status.RUNNING

			# tell the tree that you're running
			return pt.common.Status.RUNNING
		else:
			self.service_called = False
			self.finished = False
			return pt.common.Status.SUCCESS

		

class placing(pt.behaviour.Behaviour):

	"""
	Connects to place_srv.
	Performs the placing task.
	Returns running whilst awaiting the result,
	success if the action was succesful, and v.v..
	"""

	def __init__(self):

		rospy.loginfo("Initialising placing behaviour.")

		# server
		self.place_srv = rospy.get_param(rospy.get_name() + '/place_srv')

		rospy.loginfo("Connecting to place service")
		self.scene_srv = rospy.ServiceProxy(self.place_srv, SetBool)
		rospy.wait_for_service(self.place_srv, timeout=30)
		rospy.loginfo("Connected")

		# execution checker
		self.service_called = False
		self.finished = False
		self.result = SetBoolRequest()

		# become a behaviour
		super(placing, self).__init__("Placing task")

	def update(self):
		global flag

		if flag==0:


			# already pick the cube
			if self.finished: 
				return pt.common.Status.SUCCESS

			# command to pick the cube if it is not picked 
			elif not self.service_called:

				# call the service
				self.result = self.scene_srv(False)
				self.service_called = True

				# tell the tree you're running
				return pt.common.Status.RUNNING

			# if I was succesful! :)))))))))
			elif self.result.success:

				# then I'm finished!
				self.finished = True
				return pt.common.Status.SUCCESS

			# if failed
			elif not self.result.success:
				return pt.common.Status.FAILURE

			# if I'm still trying :|
			else:
				return pt.common.Status.RUNNING

			# tell the tree that you're running
			return pt.common.Status.RUNNING
		else:
			self.service_called = False
			self.finished = False
			return pt.common.Status.SUCCESS

			

class detect(pt.behaviour.Behaviour):

	"""
	Subscribes to aruco detection.
	Returns success if there is a cube,
	returns failure if not.
	"""

	def __init__(self):

		rospy.loginfo("Initialising dectect behaviour.")

		# subscriber
		self.aruco_there = rospy.get_param(rospy.get_name() + '/aruco_pose_topic')

		# execution checker
		self.detected = False

		# become a behaviour
		super(detect, self).__init__("Detecting cube")

	def aruco_there_cb(self, aruco_pose):
		self.detected = True


	def update(self):
		global flag

		if flag==0:

			self.aruco_pose_subs = rospy.Subscriber(self.aruco_there, PoseStamped, self.aruco_there_cb)

			# Cube?
			if self.detected: 
				return pt.common.Status.SUCCESS

			# not cube
			elif not self.detected:
				return pt.common.Status.FAILURE
		else:
			self.aruco_pose_subs.unregister()
			self.detected = False
			return pt.common.Status.FAILURE


class notDetect(pt.behaviour.Behaviour):

	"""
	Subscribes to aruco detection.
	Returns success if there is not a cube,
	returns failure if there is.
	"""

	def __init__(self):

		rospy.loginfo("Initialising not dectect behaviour.")

		# subscriber
		self.aruco_there = rospy.get_param(rospy.get_name() + '/aruco_pose_topic')

		# execution checker
		self.detected = False

		# become a behaviour
		super(notDetect, self).__init__("Not detecting cube")

	def aruco_there_cb(self, aruco_pose):
		self.detected = True


	def update(self):
		global flag

		if flag==0:

			self.aruco_pose_subs = rospy.Subscriber(self.aruco_there, PoseStamped, self.aruco_there_cb)

			# Not Cube
			if not self.detected: 
				return pt.common.Status.SUCCESS

			# Cube
			elif self.detected:
				return pt.common.Status.FAILURE
		else:
			self.aruco_pose_subs.unregister()
			self.detected = False
			return pt.common.Status.FAILURE



class restart(pt.behaviour.Behaviour):

	def __init__(self):

		global flag

		rospy.loginfo("Restart")

		flag = 0

		# become a behaviour
		super(restart, self).__init__("Restart BT")


	def update(self):

		global flag

		if flag == 1:
			flag = 0
			return pt.common.Status.FAILURE

		flag = 1

		return pt.common.Status.SUCCESS
		
class localization(pt.behaviour.Behaviour):

	"""
	Calls a service to know its position.
	Returns running whilst awaiting the result,
	success if the action was succesful, and v.v..
	"""

	def __init__(self):

		rospy.loginfo("Initialising localization behaviour.")

		# server
		loc_srv = rospy.get_param(rospy.get_name() + '/global_loc_srv')
		#intro loc_srv como parametro 1 y poner '/global_localization' en el launch_file
		self.global_localization = rospy.ServiceProxy('global_localization', Empty)
		rospy.wait_for_service(self.global_localization, timeout=30)

		# execution checker
		self.tried = False
		self.done = False

		# become a behaviour
		super(localization, self).__init__("Localization!")

	def update(self):
		global flag
		if flag==0:

			# success if done
			if self.done:
				return pt.common.Status.SUCCESS

			# try if not tried
			elif not self.tried:

				# command
				self.global_localization_req = self.global_localization()
				self.tried = True

				# tell the tree you're running
				return pt.common.Status.RUNNING

			# if succesful
			elif self.global_localization_req.success:
				self.done = True
				return pt.common.Status.SUCCESS

			# if failed
			elif not self.global_localization_req.success:
				return pt.common.Status.FAILURE

			# if still trying
			else:
				return pt.common.Status.RUNNING
		else:
			self.tried = False
			self.done = False
			return pt.common.Status.SUCCESS
			
class navigation(pt.behaviour.Behaviour):

	"""
	Calls a service to know its position.
	Returns running whilst awaiting the result,
	success if the action was succesful, and v.v..
	"""

	def __init__(self, goal):

		rospy.loginfo("Initialising navigation behaviour.")

		# publisher '/move_base_simple/goal'
		nav_publisher = rospy.get_param(rospy.get_name() + '/nav_goal_topic')
		#intro nav_publisher como parametro 1 y poner 'move_base_simple/goal' en el launch_file
		self.nav_goal_pub = rospy.Publisher(nav_publisher, PoseStamped, queue_size=10)
		
		self.goal = goal

		# execution checker
		self.sent = False

		# become a behaviour
		super(localization, self).__init__("Localization!")

	def update(self):
		global flag
		if flag==0:

			# success if sent
			if self.sent:
				return pt.common.Status.SUCCESS

			# send goal if not sent
			elif not self.sent:

				# command
				self.nav_goal_pub.publish(self.goal)
				self.sent = True

				# tell the tree you're running
				return pt.common.Status.RUNNING

		else:
			self.sent = False
			return pt.common.Status.SUCCESS



if __name__ == "__main__":


	rospy.init_node('main_state_machine')
	try:
		BehaviourTree()
	except rospy.ROSInterruptException:
		pass

	rospy.spin()
