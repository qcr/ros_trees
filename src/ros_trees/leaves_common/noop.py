import rospy

from ..leaves import Leaf


class Wait(Leaf):

    def __init__(self, name='Wait', *args, **kwargs):
        super(Wait, self).__init__(name, *args, **kwargs)

    def _extra_initialise(self):
        self.ts = None

    def _extra_update(self):
        if not self.ts:
            self.ts = rospy.get_time()

    def _is_leaf_done(self):
        return rospy.get_time() - self.ts > self.loaded_data
