from time import sleep
import rclpy
import message_filters
from rclpy.action import ActionClient
from actionlib_msgs.msg import GoalStatus

from . import data_management as dm
from .leaves import Leaf


class ActionLeaf(Leaf):

    def __init__(self, name, action_namespace, action_class, *args, **kwargs):
        Leaf.__init__(self, name=name, *args, **kwargs)
        self.node = None
        self.action_namespace = (action_namespace
                                 if action_namespace.startswith('/') else
                                 '/{}'.format(action_namespace))
        self._action_class = action_class
        self._action_client = None
        self.sent_goal = False

    def _default_eval_fn(self, value):
        return (self._action_client.get_state() == GoalStatus.SUCCEEDED and
                super(ActionLeaf, self)._default_eval_fn(value))

    def _default_load_fn(self, auto_generate=True):
        if auto_generate:
            return dm.auto_generate(super(ActionLeaf, self)._default_load_fn(), self._action_class.Goal)
        else:
            return super(ActionLeaf, self)._default_load_fn()

    def _default_result_fn(self):
        return self._action_client.get_result()

    def _extra_initialise(self):
        self.sent_goal = False

    def _extra_setup(self, **kwargs):
        self.node = kwargs['node']

        # Get a client for the Action Server
        try:
            self._action_client = ActionClient(
                self.node, self._action_class, self.action_namespace)
        except:
            raise Exception(
                'Failed to set up action client for server: {}'.format(
                    self.action_namespace))

        # Confirm the action client is actually there
        if not self._action_client.wait_for_server(timeout_sec=5.0):
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
        Leaf.__init__(self, name=name, *args, **kwargs)
        self.node = None
        self.topic_name = topic_name
        self.topic_class = topic_class
        self._publisher = None

    def _default_load_fn(self, auto_generate=True):
        if auto_generate:
            return dm.auto_generate(super(PublisherLeaf, self)._default_load_fn(), self.topic_class)
        else:
            return super(PublisherLeaf, self)._default_load_fn()

    def _default_result_fn(self, custom_data=None):
        try:
            self._publisher.publish(
                custom_data if custom_data is not None else self.loaded_data)
            return True
        except Exception as e:
            self.logger.error("%s.result_fn(): %s" % (self.name, e))
            return None

    def _extra_setup(self, **kwargs):
        self.node = kwargs['node']
        # Note: In ROS1 there was an issue where the publisher reference would be returned prior
        # to it connecting to ROS. This would result in dropped messages should publish be called
        # too early. TODO: Check this issue doesn't still existing in ROS2
        self._publisher = self.node.create_publisher(
            self.topic_class, self.topic_name, queue_size=10)
        return True


class ServiceLeaf(Leaf):

    def __init__(self, name, service_name, service_class, save=True, *args, **kwargs):
        Leaf.__init__(self, name=name, save=save, *args, **kwargs)
        self.node = None
        self.service_name = service_name
        self._service_class = service_class
        self._service_client = None
        self._future = None

    def _default_load_fn(self, auto_generate=True):
        if auto_generate:
            return dm.auto_generate(super(ServiceLeaf, self)._default_load_fn(), self._service_class.Request)
        else:
            return super(ServiceLeaf, self)._default_load_fn()
    
    def _extra_update(self):
        if (self._future is None):
            self._future = self._service_client.call_async(self.loaded_data)

    def _default_result_fn(self):
        result = self._future.result()
        self._future = None
        return result
        
    def _is_leaf_done(self):
        future_done = self._future is None or self._future.done()
        return super()._is_leaf_done() and future_done

    def _extra_setup(self, **kwargs):
        self.node = kwargs['node']
        # Get the service class type & a service proxy
        self._service_client = self.node.create_client(self._service_class, self.service_name)
        
        # Confirm the service is actually there
        if not self._service_client.wait_for_service(timeout_sec=5.0):
            self.logger.error("%s.setup() could not find a Service with name \"%s\"" % (self.name, self.service_name))
            return False
            
        return True


class SubscriberLeaf(Leaf):

    def __init__(self, name, topic_name, topic_class, once_only=False, expiry_time=None, timeout=3.0, save=True, *args, **kwargs):
        Leaf.__init__(self, name=name, save=save, *args, **kwargs)
        self.node = None
        self.topic_name = topic_name
        self.topic_class = topic_class
        self.once_only = once_only
        self.expiry_time = expiry_time
        self.timeout = timeout
        self._subscriber = None
        self._cached_data = None
        self._cached_time = None
        self._last_msg = None

    def _default_result_fn(self):
        t = self.node.get_clock().now()

        # While we don't have a valid message, keep waiting
        while ((self._cached_time is None or
            (self.expiry_time is not None and
            self.node.get_clock().now() - self._cached_time > self.expiry_time)) and
            self.node.get_clock().now() - t < self.timeout):
            sleep(0.1)

        # Return what we found, dumping the cache if we're returning something
        # in once_only mode
        out = (None if self._cached_time is None or
               (self.expiry_time and
                self.node.get_clock().now() - self._cached_time > self.expiry_time) else
               self._cached_data)
        if self.once_only and out:
            self._cached_data = None
            self._cached_time = None
        return out

    def _extra_setup(self, **kwargs):
        self.node = kwargs['node']
        self._subscriber = self.node.create_subscription(self.topic_class, self.topic_name, self.callback)
        return True

    def callback(self, msg):
        self._cached_data = msg
        self._cached_time = self.node.get_clock().now()


class SyncedSubscriberLeaf(Leaf):

    def __init__(self, name, topic_names, topic_classes, expiry_time=None, timeout=3.0, save=True, *args, **kwargs):
        Leaf.__init__(self, name=name, save=save, *args, **kwargs)

        if not (isinstance(topic_names, list) or isinstance(topic_names, tuple)) or len(topic_names) != len(topic_classes):
            raise ValueError('topic_names length does not equal topic_classes length')

        self.node = None
        self.topic_names = topic_names
        self.topic_classes = topic_classes
        self.expiry_time = expiry_time
        self.timeout = timeout
        self._filter = None
        self._subscribers = [None] * len(topic_names)
        self._cached_data = [None] * len(topic_names)
        self._cached_time = None

    def _default_result_fn(self):
        t = self.node.get_clock().now()
        while ((self._cached_time is None or
                (self.expiry_time is not None and
                 self.node.get_clock().now() - self._cached_time > self.expiry_time)) and
               self.node.get_clock().now() - t < self.timeout):
            sleep(0.1)
        return (None if self._cached_time is None or
                (self.expiry_time and
                 self.node.get_clock().now() - self._cached_time > self.expiry_time) else
                self._cached_data)

    def _extra_setup(self, **kwargs):
        self.node = kwargs['node']
        self._subscribers = [
            message_filters.Subscriber(self.node, self.topic_classes[idx], topic_name)
            for idx, topic_name in enumerate(self.topic_names)
        ]
        self._filter = message_filters.ApproximateTimeSynchronizer(
            self._subscribers, 10, 0.1, allow_headerless=True)
        self._filter.registerCallback(self.callback)
        return True

    def callback(self, *msgs):
        for idx, msg in enumerate(msgs):
            self._cached_data[idx] = msg
        self._cached_time = self.node.get_clock().now()
