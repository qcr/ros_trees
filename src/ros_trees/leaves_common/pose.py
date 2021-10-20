import rospy
from tf.transformations import quaternion_from_euler, quaternion_multiply
import tf2_ros
import tf2_geometry_msgs

from geometry_msgs.msg import Quaternion, PoseStamped

from ..leaves import Leaf


class TranslatePose(Leaf):

    def __init__(self,
                 name="Translate Pose",
                 x=0,
                 y=0,
                 z=0,
                 rx=0,
                 ry=0,
                 rz=0,
                 height_fn=None,
                 save=True,
                 *args,
                 **kwargs):
        super(TranslatePose, self).__init__(name,
                                            result_fn=self.result_fn,
                                            save=save,
                                            *args,
                                            **kwargs)
        self.x = x
        self.y = y
        self.z = z

        self.rx = rx
        self.ry = ry
        self.rz = rz

        self.height_fn = (self.default_height_fn if height_fn is None else
                          self._ensure_bound(height_fn))

    def default_height_fn(self):
        return self.z

    def result_fn(self):
        result = self._default_result_fn()

        if not isinstance(result, PoseStamped):
            for key in result.__slots__:
                if isinstance(getattr(result, key), PoseStamped):
                    result = getattr(result, key)
                    break

        print(result)
        result.pose.position.x += self.x
        result.pose.position.y += self.y
        result.pose.position.z += self.height_fn()

        current = self.quaternion_to_list(result.pose.orientation)
        rotation = quaternion_from_euler(self.rx, self.ry, self.rz)

        rotated = quaternion_multiply(current, rotation)

        result.pose.orientation = self.list_to_quaternion(rotated)
        print(result)
        return result

    def quaternion_to_list(self, quaternion):
        return [quaternion.x, quaternion.y, quaternion.z, quaternion.w]

    def list_to_quaternion(self, l):
        q = Quaternion()
        q.x = l[0]
        q.y = l[1]
        q.z = l[2]
        q.w = l[3]
        return q


class TransformPose(Leaf):

    def __init__(self,
                 name="Transform Pose",
                 target_frame='base_link',
                 save=True,
                 *args,
                 **kwargs):
        super(TransformPose, self).__init__(name,
                                            result_fn=self.result_fn,
                                            save=save,
                                            *args,
                                            **kwargs)
        self.target_frame = target_frame

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)

        rospy.sleep(1)

    def result_fn(self):
        result = self._default_result_fn()

        if not isinstance(result, PoseStamped):
            for key in result.__slots__:
                if isinstance(getattr(result, key), PoseStamped):
                    result = getattr(result, key)
                    break

        tf = self._tf_buffer.lookup_transform(self.target_frame,
                                              result.header.frame_id,
                                              rospy.Duration(0.0))
        return tf2_geometry_msgs.do_transform_pose(result, tf)
