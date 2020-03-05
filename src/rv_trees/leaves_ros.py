import actionlib
from actionlib_msgs.msg import GoalStatus
import re
import roslib
import rospy
import rosservice
import rostopic
import message_filters

from . import data_management as dm
from .leaves import Leaf


class ActionLeaf(Leaf):

    def __init__(self, name, action_namespace, *args, **kwargs):
        super(ActionLeaf, self).__init__(name, *args, **kwargs)
        self.action_namespace = action_namespace if action_namespace.startswith('/') else '/{}'.format(action_namespace)
        self._action_class = None
        self._action_client = None
        self.sent_goal = False

    def _default_eval_fn(self, value):
        return (self._action_client.get_state() == GoalStatus.SUCCEEDED and
                super(ActionLeaf, self)._default_eval_fn(value))

    def _default_load_fn(self):
        return dm.auto_generate(
            super(ActionLeaf, self)._default_load_fn(),
            type(self._action_class().action_goal.goal))

    def _default_result_fn(self):
        return self._action_client.get_result()

    def _extra_initialise(self):
        self.sent_goal = False

    def _extra_setup(self, timeout):
        # Get a client & type for the Action Server
        self._action_class = roslib.message.get_message_class(
            re.sub('Goal$', '',
                   rostopic.get_topic_type(self.action_namespace +
                                           '/goal')[0]))
        self._action_client = actionlib.SimpleActionClient(
            self.action_namespace, self._action_class)

        # Confirm the action client is actually there
        if not self._action_client.wait_for_server(rospy.Duration(timeout)):
            self.logger.error(
                "%s.setup() could connect to Action Server with name \"%s\"" %
                (self.name, self.action_namespace))
            self._action_client = None
            return False
        return True

    def _extra_terminate(self, new_status):
        if (self._action_client is not None and self.sent_goal and
                self._action_client.get_state() in [
                    GoalStatus.PENDING, GoalStatus.ACTIVE,
                    GoalStatus.PREEMPTING, GoalStatus.RECALLING
                ]):
            self._action_client.cancel_goal()
        self.sent_goal = False

    def _extra_update(self):
        if not self.sent_goal:
            self._action_client.send_goal(self.loaded_data)
            self.sent_goal = True
        return None

    def _is_leaf_done(self):
        return self._action_client.get_result() is not None


class PublisherLeaf(Leaf):

    def __init__(self, name, topic_name, topic_class, *args, **kwargs):
        super(PublisherLeaf, self).__init__(name, *args, **kwargs)
        self.topic_name = topic_name
        self.topic_class = topic_class
        self._publisher = None

    def _default_load_fn(self):
        return dm.auto_generate(
            super(PublisherLeaf, self)._default_load_fn(), self.topic_class)

    def _default_result_fn(self):
        try:
            self._publisher.publish(self.loaded_data)
            return True
        except Exception as e:
            self.logger.error("%s.result_fn(): %s" % (self.name, e))
            return None

    def _extra_setup(self, timeout):
        # Annoyingly have to do this stupid code check because when you declare
        # a publisher the code returns before it is setup (so the tree can end
        # up calling publish before it is ready & the message gets lost... nice
        # one ROS...)
        t = rospy.get_time()
        sub = rospy.Subscriber(self.topic_name, self.topic_class)
        
        self._publisher = rospy.Publisher(self.topic_name,
                                          self.topic_class,
                                          queue_size=10)
        
        while (self._publisher.get_num_connections() == 0 and
            (timeout == 0 or rospy.get_time() - t < timeout)):
            rospy.sleep(0.05)
        
        return self._publisher.get_num_connections() > 0


class ServiceLeaf(Leaf):

    def __init__(self, name, service_name, save=True, *args, **kwargs):
        super(ServiceLeaf, self).__init__(name, save=save, *args, **kwargs)
        self.service_name = service_name
        self._service_class = None
        self._service_proxy = None

    def _default_load_fn(self):
        return dm.auto_generate(
            super(ServiceLeaf, self)._default_load_fn(),
            self._service_class._request_class)

    def _default_result_fn(self):
        try:
            return self._service_proxy(self.loaded_data)
        except Exception as e:
            self.logger.error("%s.result_fn(): %s" % (self.name, e))
            return None

    def _extra_setup(self, timeout):
        # Confirm the service is actually there
        try:
            rospy.wait_for_service(self.service_name, timeout if timeout else None)
        except rospy.ROSException:
            self.logger.error(
                "%s.setup() could not find a Service with name \"%s\"" %
                (self.name, self.service_name))
            return False

        # Get the service class type & a service proxy
        self._service_class = rosservice.get_service_class_by_name(
            self.service_name)
        self._service_proxy = rospy.ServiceProxy(self.service_name,
                                                 self._service_class)
        return True


class SubscriberLeaf(Leaf):

    def __init__(self,
                 name,
                 topic_name,
                 topic_class,
                 expiry_time=None,
                 timeout=3.0,
                 save=True,
                 *args,
                 **kwargs):
        super(SubscriberLeaf, self).__init__(name, save=save, *args, **kwargs)
        self.topic_name = topic_name
        self.topic_class = topic_class
        self.expiry_time = expiry_time
        self.timeout = timeout
        self._subscriber = None
        self._cached_data = None
        self._cached_time = None

    def _default_result_fn(self):
        t = rospy.get_time()
        while ((self._cached_time is None or
                (self.expiry_time is not None and
                 rospy.get_time() - self._cached_time > self.expiry_time)) and
               rospy.get_time() - t < self.timeout):
            rospy.sleep(0.1)
        return (None if self._cached_time is None or
                (self.expiry_time and rospy.get_time() - self._cached_time > self.expiry_time) else
                self._cached_data)

    def _extra_setup(self, timeout):
        self._subscriber = rospy.Subscriber(self.topic_name, self.topic_class,
                                            self.callback)
        return True

    def callback(self, msg):
        self._cached_data = msg
        self._cached_time = rospy.get_time()

class SyncedSubscriberLeaf(Leaf):
    def __init__(self,
                 name,
                 topic_names,
                 topic_classes,
                 expiry_time=None,
                 timeout=3.0,
                 save=True,
                 *args,
                 **kwargs):
        super(SyncedSubscriberLeaf, self).__init__(name, save=save, *args, **kwargs)

        if not (isinstance(topic_names, list) or isinstance(topic_names, tuple)) or len(topic_names) != len(topic_classes):
            raise ValueError('topic_names length does not equal topic_classes length')

        self.topic_names = topic_names
        self.topic_classes = topic_classes
        self.expiry_time = expiry_time
        self.timeout = timeout
        self._filter = None
        self._subscribers = [None] * len(topic_names)
        self._cached_data = [None] * len(topic_names)
        self._cached_time = None

    def _default_result_fn(self):
        t = rospy.get_time()
        while ((self._cached_time is None or
                (self.expiry_time is not None and
                 rospy.get_time() - self._cached_time > self.expiry_time)) and
               rospy.get_time() - t < self.timeout):
            rospy.sleep(0.1)
        return (None if self._cached_time is None or
                (self.expiry_time and rospy.get_time() - self._cached_time > self.expiry_time) else
                self._cached_data)

    def _extra_setup(self, timeout):
        self._subscribers = [ message_filters.Subscriber(topic_name, self.topic_classes[idx]) 
            for idx, topic_name in enumerate(self.topic_names) ]
        self._filter = message_filters.ApproximateTimeSynchronizer(self._subscribers, 10, 0.1, allow_headerless=True)
        self._filter.registerCallback(self.callback)
        return True

    def callback(self, *msgs):
        for idx, msg in enumerate(msgs):
            self._cached_data[idx] = msg
        self._cached_time = rospy.get_time()

