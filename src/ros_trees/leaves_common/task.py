from ros_trees.leaves_ros import SubscriberLeaf
from std_msgs.msg import String

class IsTaskSelected(SubscriberLeaf):
  def __init__(self, task_name, topic_name='/task', topic_class=String, *args, **kwargs):
    super(IsTaskSelected,
          self).__init__('Task Selected'.format(task_name),
                          topic_name=topic_name, 
                          topic_class=topic_class,
                          eval_fn=self.eval_fn,
                          timeout=0.001,
                          *args, 
                          **kwargs)
    self.task_name = task_name

  def eval_fn(self, value):
    return (not value and self.task_name == '') or (value and value.data == self.task_name)
    