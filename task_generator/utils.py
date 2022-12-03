import rospy
import os


class Utils:
    def get_environment():
        return rospy.get_param("environment", "arena2d").lower()
    
    def get_arena_type():
        return os.getenv("ARENA_TYPE", "training").lower()
