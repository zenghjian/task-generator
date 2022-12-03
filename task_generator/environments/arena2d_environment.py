import rospy
from std_srvs.srv import Empty
from .base_environment import BaseEnvironment
from .environment_factory import EnvironmentFactory

from geometry_msgs.msg import PoseStamped, Pose2D, Point
from arena2d_msgs.srv import (
    DeleteModel,
    MoveModel,
    SpawnModel,
    SpawnModelRequest
)

from ..constants import Constants, FlatlandRandomModel

T = Constants.WAIT_FOR_SERVICE_TIMEOUT

@EnvironmentFactory.register("arena2d")
class Arena2dEnviorment(BaseEnvironment):
    def __init__(self, namespace):
        super().__init__(namespace)
        self._namespace = namespace
        self._ns_prefix = "arena2d/"

        rospy.wait_for_service(f"{self._ns_prefix}move_model", timeout=T)
        rospy.wait_for_service(f"{self._ns_prefix}spawn_model", timeout=T)
        rospy.wait_for_service(f"{self._ns_prefix}delete_model", timeout=T)
        rospy.wait_for_service(f"{self._ns_prefix}pause", timeout=T)
        rospy.wait_for_service(f"{self._ns_prefix}unpause", timeout=T)

        self._move_model_srv = rospy.ServiceProxy(
            f"{self._ns_prefix}move_model", MoveModel
        )
        self._spawn_model_srv = rospy.ServiceProxy(
            f"{self._ns_prefix}spawn_model", SpawnModel
        )
        self._delete_model_srv = rospy.ServiceProxy(
            f"{self._ns_prefix}delete_model", DeleteModel
        )

        self._pause_srv = rospy.ServiceProxy(
            f"{self._ns_prefix}pause", Empty
        )
        self._unpause_srv = rospy.ServiceProxy(
            f"{self._ns_prefix}unpause", Empty
        )

        self._obstacles_amount = 0

        self.count_dynamic = 0
        self.count_static = 0

    def before_reset_task(self):
        self._pause_srv()
        pass

    def after_reset_task(self):
        rospy.loginfo("call dynamic obstacle: %d " %(self.count_dynamic))
        rospy.loginfo("call static obstacle: %d " %(self.count_static))
        self._unpause_srv()

    def remove_all_obstacles(self):
        self._delete_model_srv("all")
        self._obstacles_amount = 0


        self.count_dynamic =0
        self.count_static =0
        
    def spawn_random_dynamic_obstacle(self, **args):
        """
        Spawn a single random dynamic obstacle.
        
        Args:
            position: [int, int, int] denoting the x, y and angle.
            min_radius: minimal radius of the obstacle
            max_radius: maximal radius of the obstacle
            linear_vel: linear velocity
            angular_vel_max: maximal angular velocity
        """
        spawn_model_request = SpawnModelRequest()
        spawn_model_request.name = "dynamic"
        spawn_model_request.min_radius = FlatlandRandomModel.MAX_RADIUS
        spawn_model_request.max_radius = FlatlandRandomModel.MAX_RADIUS # Uniform size
        spawn_model_request.linear_vel = FlatlandRandomModel.LINEAR_VEL
        spawn_model_request.angular_vel_max = FlatlandRandomModel.ANGLUAR_VEL_MAX
        self._spawn_model_srv(spawn_model_request)
        self._obstacles_amount += 1

        self.count_dynamic += 1

    def spawn_random_static_obstacle(self, **args):
        """
        Spawn a single random static obstacle.
        
        Args:
            position: [int, int, int] denoting the x, y and angle.
            min_radius: minimal radius of the obstacle
            max_radius: maximal radius of the obstacle
        """
        spawn_model_request = SpawnModelRequest()
        spawn_model_request.name = "static"
        spawn_model_request.min_radius = FlatlandRandomModel.MIN_RADIUS
        spawn_model_request.max_radius = FlatlandRandomModel.MAX_RADIUS
        spawn_model_request.linear_vel = 0.0
        spawn_model_request.angular_vel_max = 0.0
        self._spawn_model_srv(spawn_model_request)
        self._obstacles_amount += 1

        self.count_static += 1

    def publish_goal(self, goal):
        """
        Publishes the goal. 
        """
        pass

    def move_robot(self, pos, name=None):
        
        name = "robot"
        pose = Pose2D()
        pose.x = pos[0]
        pose.y = pos[1]
        pose.theta = pos[2]
        self._move_model_srv(name, pose)

    def spawn_robot(self, name, robot_name, namespace_appendix=None):
        """
        In arena2d the robot generated at Initalization
        """
        pass

    def spawn_pedsim_agents(self, agents):
        pass

    def reset_pedsim_agents(self):
        pass

    def spawn_obstacle(self, position, yaml_path=""):
        pass
