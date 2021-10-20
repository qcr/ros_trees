import py_trees.behaviour as behaviour
import py_trees.common as common

from py_trees.composites import Parallel
from py_trees.common import Status


class MemoryParallel(Parallel):
    # TODO see if this can be done more robustly than copy-paste-tweak from
    # Parallel composite

    def __init__(self,
                 name="Memory Parallel",
                 policy=common.ParallelPolicy.SUCCESS_ON_ALL,
                 children=None,
                 *args,
                 **kwargs):
        super(MemoryParallel, self).__init__(name,
                                             children=children,
                                             *args,
                                             **kwargs)

    def tick(self):
        if self.status != Status.RUNNING:
            # subclass (user) handling
            self.initialise()
        self.logger.debug("%s.tick()" % self.__class__.__name__)
        # process them all first
        for child in self.children:
            if child.status == Status.SUCCESS:
                continue

            for node in child.tick():
                yield node
        # new_status = Status.SUCCESS if self.policy == common.ParallelPolicy.SUCCESS_ON_ALL else Status.RUNNING
        new_status = Status.RUNNING
        if any([c.status == Status.FAILURE for c in self.children]):
            new_status = Status.FAILURE
        else:
            if self.policy == common.ParallelPolicy.SUCCESS_ON_ALL:
                if all([c.status == Status.SUCCESS for c in self.children]):
                    new_status = Status.SUCCESS
            elif self.policy == common.ParallelPolicy.SUCCESS_ON_ONE:
                if any([c.status == Status.SUCCESS for c in self.children]):
                    new_status = Status.SUCCESS
        # special case composite - this parallel may have children that are still running
        # so if the parallel itself has reached a final status, then these running children
        # need to be made aware of it too
        if new_status != Status.RUNNING:
            for child in self.children:
                if child.status == Status.RUNNING:
                    # interrupt it (exactly as if it was interrupted by a higher priority)
                    child.stop(Status.INVALID)
            self.stop(new_status)
        self.status = new_status
        yield self
