#!/usr/bin/env python

from math import *

import py_trees as pt
import py_trees_ros as ptr
import rospy
from actionlib import SimpleActionClient
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from move_base_msgs.msg import MoveBaseActionFeedback
from numpy import linalg
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal
from robotics_project.srv import MoveHead, MoveHeadRequest, MoveHeadResponse
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Empty, SetBool, SetBoolRequest

from reactive_sequence import RSequence

flag = 0
flag_kidnp = True

def get_yaw( q):

    return atan2(2 * (q.w * q.z + q.x * q.y),
                    1 - 2 * (q.y * q.y + q.z * q.z))

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

		"""
		# become the treefrom geometry_msgs.msg import PoseStamped (PART C)
		tree = RSequence(name="Main sequence", children=[b0, b1])
		super(BehaviourTree, self).__init__(tree)
		"""
		
		# tuck the arm
		a01 = tuckarm()

		# rise head
		a02 = movehead("up")

		a0 = RSequence(
			name="Initialization",
			children=[a01, a02]
		)

		a10 = knownPosition()

		a11 = localisation()

		a12 = pt.composites.Selector(
			name="Rotate fallback",
			children=[counter(60, "At table?"), go("Rotate!", 0, 1)]
		)

		c5 = counter2(100, "At table?")

		a1 = pt.composites.Selector(
			name="Localization fallback",
			children=[a10, a11]
		)

		c6 = RSequence(
			name="Localization sequence",
			children=[a1, a12]
		)

		a2 = navigation("pick")

		c0 = checkPosition("pick")

		a52 = navigation("pick")

		c2 = checkPosition("pick")

		c4 = pt.composites.Selector(
			name="Reach position fallback",
			children=[c2, b63]
		)

		a53 = movehead("up")

		a51 = RSequence(
			name="Back to table sequence",
			children=[b60, a53, a52, b63]
		)

		a5 = pt.composites.Selector(
			name="Finish task?",
			children=[b5, a51]
		)

		a43 = movehead("down")

		a42 = navigation("place")

		a41 = movehead("up")

		c1 = checkPosition("place")

		c3 = pt.composites.Selector(
			name="Reach position fallback",
			children=[c1, b4]
		)

		a44 = RSequence(
			name="Complete pick and place sequence",
			children=[b2, a41, a42, a43, b4, a5]
		)

		a4 = pt.composites.Selector(
			name="Detecting fallback",
			children=[b10, a44]
		)

		a6 = RSequence(
			name="Pick and place sequence", 
			children=[b02, a4]
			)

		a7 = pt.composites.Selector(
			name = "Reach position fallback",
			children=[c0, a6]
		)

		
		# become the treefrom geometry_msgs.msg import PoseStamped
		tree = RSequence(name="Main sequence", children=[a0, c6, c5, a2, a6])
		super(BehaviourTree, self).__init__(tree)
		
		
		# execute the behaviour tree
		rospy.sleep(5)
		self.setup(timeout=10000)
		while not rospy.is_shutdown(): 
			self.tick_tock(1)

class knownPosition(pt.behaviour.Behaviour):

	"""
	Checks if the robot knows where it is.
	Returns success if the robot knows it position,
	returns failure if not
	"""

	def __init__(self):

		self.amcl_pose_subs = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.amcl_cb)

		# execution checker
		self.known = False
		self.flag_cov = 0

		# become a behaviour
		super(knownPosition, self).__init__("Known Position?")

	def amcl_cb(self, data):
		self.acml_position = data

	def update(self):
		global flag
		global flag_kidnp

		if flag == 0:

			#print("Covariance", linalg.norm(self.acml_position.pose.covariance))
			error = linalg.norm(self.acml_position.pose.covariance)
			if error < 0.025:
				if self.known == False:
					self.flag_cov = 0
				self.known = True

				if self.flag_cov == 0:
					self.flag_cov = 1
					print("Found: ", error)

			if error > 0.035:
				if self.known == True:
					flag_kidnp = True
					self.flag_cov = 0
				self.known = False

				if self.flag_cov == 0:
					self.flag_cov = 1
					print("Found: ", error)
				

			# success if done
			if self.known:
				return pt.common.Status.SUCCESS

			# try if not tried
			elif not self.known:
				return pt.common.Status.FAILURE
		else:
			self.known = False
			return pt.common.Status.FAILURE


class checkPosition(pt.behaviour.Behaviour):

	"""
	
	"""
	def __init__(self, name):

		self.amcl_pose_subs = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.amcl_cb)
		self.name = name

		if self.name == "pick":
			self.pick_pose = rospy.get_param(rospy.get_name() + '/pick_pose_topic')
			self.pick_pose_subs = rospy.Subscriber(self.pick_pose, PoseStamped, self.data_cb)

		elif self.name == "place":
			self.place_pose = rospy.get_param(rospy.get_name() + '/place_pose_topic')
			self.place_pose_subs = rospy.Subscriber(self.place_pose, PoseStamped, self.data_cb)

		# execution checker
		self.done = False

		# become a behaviour
		super(checkPosition, self).__init__("checkPosition!")

	def amcl_cb(self, data):
		self.data = data

	def data_cb(self, data):
		self.goal = data
		

	def update(self):
		global flag
		if flag==0:

			self.goal_yaw  = get_yaw(self.goal.pose.orientation)
			self.robot_yaw  = get_yaw(self.data.pose.pose.orientation)
			if hypot(self.data.pose.pose.position.x - self.goal.pose.position.x, self.data.pose.pose.position.y-self.goal.pose.position.y) < 0.06 and fabs(self.robot_yaw-self.goal_yaw) < 0.02 and linalg.norm(self.data.pose.covariance) < 0.03:
				print("error pos", hypot(self.data.pose.pose.position.x - self.goal.pose.position.x, self.data.pose.pose.position.y-self.goal.pose.position.y))
				print("Error yaw", fabs(self.robot_yaw-self.goal_yaw))
				print("covariance", linalg.norm(self.data.pose.covariance))
				return pt.common.Status.FAILURE
			else:
				return pt.common.Status.SUCCESS
		
		else:
			return pt.common.Status.FAILURE


class localisation(pt.behaviour.Behaviour):
	"""
	Orders the robot to localize itself
	"""
	def __init__(self):

		self.loc_srv = rospy.get_param(rospy.get_name() + '/global_loc_srv')
		rospy.wait_for_service(self.loc_srv,timeout=30)
		self.global_localization = rospy.ServiceProxy(self.loc_srv, Empty)

		# execution checker
		self.done = False

		# become a behaviour
		super(localisation, self).__init__("Localization!")

	def update(self):
		global flag
		global flag_kidnp

		if flag==0:

			if flag_kidnp == True:
				self.done = False
				flag_kidnp = False
			# success if done
			if self.done:
				return pt.common.Status.SUCCESS

			# command
			self.global_localization_req = self.global_localization()
			self.done = True

			# tell the tree you're running
			return pt.common.Status.RUNNING
			
		else:
			self.done = False
			return pt.common.Status.SUCCESS


class navigation(pt.behaviour.Behaviour):

	"""
	Calls a service to know its position.
	Returns running whilst awaiting the result,
	success if the action was succesful, and v.v..
	"""

	def __init__(self, action):

		rospy.loginfo("Initialising navigation behaviour.")

		# publisher '/move_base_simple/goal'
		self.nav_publisher = rospy.get_param(rospy.get_name() + '/nav_goal_topic')
		self.nav_goal_pub = rospy.Publisher(self.nav_publisher, PoseStamped, queue_size=10)
		
		self.pick_pose = rospy.get_param(rospy.get_name() + '/pick_pose_topic')
		self.place_pose = rospy.get_param(rospy.get_name() + '/place_pose_topic')
		self.pick_pose_subs = rospy.Subscriber(self.pick_pose, PoseStamped, self.pick_cb)
		self.place_pose_subs = rospy.Subscriber(self.place_pose, PoseStamped, self.place_cb)

		self.amcl_pose_subs = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.amcl_cb)
		self.fdbk_subs = rospy.get_param(rospy.get_name() + '/move_base_feedback')
		#self.feedback_subs = rospy.Subscriber(self.fdbk_publisher, MoveBaseActionFeedback, self.fdbk_cb)

		self.action = action

		# execution checker
		self.finished = False
		self.sent = False

		# become a behaviour
		super(navigation, self).__init__("Navigation!")

	def pick_cb(self, data):
		self.pick = data

	def place_cb(self, data):
		self.place = data

	def amcl_cb(self, data):
		self.data = data

	def update(self):
		global flag
		if flag==0:

			# success if sent
			if self.finished:
				return pt.common.Status.SUCCESS

			# send goal if not sent
			if not self.sent:

				# command
				if self.action == "pick":
					self.nav_goal_pub.publish(self.pick)
					self.goal = self.pick
				elif self.action == "place":
					self.nav_goal_pub.publish(self.place)
					self.goal = self.place
				self.sent = True

				# tell the tree you're running
				return pt.common.Status.RUNNING
							
			# if I'm still trying :|
			else:

				try: 
					rospy.wait_for_message(self.fdbk_subs, MoveBaseActionFeedback, timeout=4)
					return pt.common.Status.RUNNING
				except rospy.exceptions.ROSException:
					x = self.data.pose.pose.position.x - self.goal.pose.position.x
					y = self.data.pose.pose.position.y - self.goal.pose.position.y
					z = self.data.pose.pose.position.z - self.goal.pose.position.z
					xo = self.data.pose.pose.orientation.x - self.goal.pose.orientation.x
					yo = self.data.pose.pose.orientation.y - self.goal.pose.orientation.y
					zo = self.data.pose.pose.orientation.z - self.goal.pose.orientation.z
					w = self.data.pose.pose.orientation.w - self.goal.pose.orientation.w
					if(pow(x,2) + pow(y,2) + pow(z,2) + pow(xo,2) + pow(yo,2) + pow(zo,2) +pow(w,2))<0.05:
						self.finished = True
						return pt.common.Status.SUCCESS
					else:
						self.finished = False
						self.sent = False
						return pt.common.Status.FAILURE


		else:
			self.finished = False
			self.sent = False
			return pt.common.Status.SUCCESS

			
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
		global flag, flag_kidnp

		if flag == 0 :

			# increment i
			self.i += 1

			# succeed after count is done
			return pt.common.Status.FAILURE if self.i <= self.n else pt.common.Status.SUCCESS
		
		else:
			self.i = 0
			flag_kidnp = 0
			return pt.common.Status.SUCCESS


class counter2(pt.behaviour.Behaviour):

	"""
	Returns running for n ticks and success thereafter.
	"""

	def __init__(self, n, name):

		rospy.loginfo("Initialising counter behaviour.")

		# counter
		self.i = 0
		self.n = n

		# become a behaviour
		super(counter2, self).__init__(name)

	def update(self):
		global flag

		if flag == 0:

			# increment i
			self.i += 1

			# succeed after count is done
			return pt.common.Status.RUNNING if self.i <= self.n else pt.common.Status.SUCCESS
		
		else:
			self.i = 0
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
        self.cmd_vel_pub = rospy.Publisher(self.cmd_vel_top, Twist, queue_size=1)
        
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

		if flag==1:
			flag=0
			return pt.common.Status.FAILURE

		flag = 1

		return pt.common.Status.SUCCESS



if __name__ == "__main__":


	rospy.init_node('main_state_machine')
	try:
		BehaviourTree()
	except rospy.ROSInterruptException:
		pass

	rospy.spin()
