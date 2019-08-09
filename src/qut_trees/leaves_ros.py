import rospy
import rosservice

import data_management as dm
from leaves import Leaf


class ServiceLeaf(Leaf):

    def __init__(self, name, service_name, *args, **kwargs):
        super(ServiceLeaf, self).__init__(name, *args, **kwargs)
        self.service_name = service_name

    def _default_load_fn(self):
        data = super(ServiceLeaf, self)._default_load_fn()
        return self._service_class._request_class(
            *([] if data is None else dm.to_fields_list(data)))

    def _default_result_fn(self):
        try:
            return self._service_proxy(self.loaded_data)
        except Exception as e:
            self.logger.error("%s.result_fn(): %s" % (self.name, e))
            return None

    def _extra_setup(self, timeout):
        # Confirm the service is there, & get a proxy to it
        try:
            rospy.wait_for_service(self.service_name, timeout)
        except rospy.ROSException:
            self.logger.error(
                "%s.setup() could not find a service with name \"%s\"" %
                (self.name, self.service_topic))
            return False

        # Get the service class type & a service proxy
        self._service_class = rosservice.get_service_class_by_name(
            self.service_name)
        self._service_proxy = rospy.ServiceProxy(self.service_name,
                                                 self._service_class)
        return True
