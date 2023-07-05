import rospy
import traceback
import actionlib
from abc import ABC, abstractmethod


class ActionServer(ABC):
    def __init__(self, name, action_type):
        self._server = actionlib.SimpleActionServer(name, action_type, execute_cb=self._execute_cb, auto_start=False)
        self._server.start()

    def _execute_cb(self, goal):
        try:
            result = self.on_request(goal)
        except ActionError as e:
            rospy.logerr(str(e))
            self._server.set_aborted(text=str(e))
        except Exception as e:
            rospy.logerr(traceback.format_exc())
            self._server.set_aborted(text=str(e))
            rospy.signal_shutdown(str(e))
        else:
            self._server.set_succeeded(result)

    @abstractmethod
    def on_request(self, goal):
        """
        Abstract method to be implemented by subclasses. This method is called when a request is received by the action
        server.

        :param goal: The goal message received by the action server.
        :return: The result of the request.
        """
        pass


class ActionError(Exception):
    pass
