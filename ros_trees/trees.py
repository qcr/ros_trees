import datetime
import os
import py_trees as pt
import py_trees_ros as ptr
import select

try:
    import subprocess32 as subprocess
except ImportError:
    import subprocess

import sys
import time
from timeit import default_timer as timer

from . import leaves

try:
    input = raw_input
except NameError:
    pass

_TMP_LOCATION = '/tmp/ros_trees'


def _validate_tmp():
    # Ensures we can write files to tmp location
    p = os.path.abspath(_TMP_LOCATION)
    if os.path.exists(p) and not os.path.isdir(p):
        os.remove(p)
    if not os.path.exists(p):
        os.mkdir(p)


class BehaviourTree(ptr.trees.BehaviourTree):
    _LOG_LEVELS = ['INFO', 'DEBUG', 'WARN', 'ERROR']
    _current = None

    def __init__(self, tree_name, root):
        if not BehaviourTree._current is None:
            raise Exception('Multiple instances of BehaviourTree not supported.')

        super(BehaviourTree, self).__init__(root)
        self.tree_name = tree_name

        # Configure the tree to capture keyboard input & pass it down to leaves
        # that are in the debugging mode that awaits input
        self._cached_input_leaves = self._get_debug_input_leaves()
        self.add_pre_tick_handler(self._pre_tick)

        BehaviourTree._current = self

    def _cleanup(self):
        # We extend their cleanup method (run at shutdown) because it didn't
        # seem to cleanly kill the last running node (e.g. cancelling actions
        # in Action Servers)
        super(BehaviourTree, self)._cleanup()
        self.destroy()

    def _get_debug_input_leaves(self):
        return [
            l for l in self._get_leaves_list(unique=False)
            if isinstance(l, leaves.Leaf) and 'INPUT' in l.debug.name
        ]

    def _get_leaves_list(self, unique=True):
        # Step down the tree to get all leaves
        ls = []
        to_process = [self.root]
        while to_process:
            element = to_process.pop()
            if hasattr(element, 'children') and element.children:
                to_process.extend(element.children)
            elif hasattr(element, 'child'):
                to_process.append(element.child)
            elif isinstance(element, pt.behaviour.Behaviour):
                ls.append(element)

        # Return the list (trimming duplicates if requested)
        return list({l.name: l for l in ls}.values()) if unique else ls

    def _pre_tick(self, tree):
        # Bail if we don't have an input
        i, _, _ = select.select([sys.stdin], [], [], 0)
        if not i:
            return

        # Consume the input, & trigger leaves in the appropriate debug mode
        # TODO maybe should only set _debug_advance for the tip()?
        sys.stdin.readline()
        for l in self._cached_input_leaves:
            l._debug_advance = True

    def dump_tree_graph(self,
                        full_path=None,
                        open_graph=True,
                        open_image_type='svg'):
        if full_path is None:
            _validate_tmp()
            full_path = os.path.join(
                _TMP_LOCATION,
                datetime.datetime.now().strftime('%Y%m%d_%H%M%S'))
        pt.display.render_dot_tree(self.root, name=full_path)
        if open_graph:
            subprocess.run(["xdg-open", full_path + '.' + open_image_type])

    def print_requirements(self):
        ls = self._get_leaves_list()
        print("\nRequirements list:")
        for l in ls:
            print("\t%s (%s):" % (l.name, l.__class__.__name__))
            print_fn = getattr(l, '_requirements_str', None)
            req_str = None if print_fn is None else print_fn()
            if req_str is None:
                req_str = "No requirements declared"
            print("\n".join(["\t\t%s" % s for s in req_str.split("\n")]))

    def run(self, hz=10, push_to_start=True, log_level=None, setup_timeout=5, exit_on=None):
        # Configure the requested log_level
        if log_level:
            log_level = log_level.upper()

            if not log_level in BehaviourTree._LOG_LEVELS:
                raise ValueError("Provided log_level \'%s\' is not supported. "
                                "Supported values are: %s" %
                                (log_level, BehaviourTree._LOG_LEVELS))
        
            pt.logging.level = pt.logging.Level[log_level]

        # TODO should maybe not do setup every time... but eh
        try:
            self.setup(timeout=setup_timeout)
        except Exception as e:
            self.root.logger.error(
                "Failed to setup the \"%s\" tree. Aborting run..." %
                self.tree_name)
            self.root.logger.error(e)
            return False

        # Run the tree indefinitely
        if push_to_start:
            input("Press Enter to start the \"%s\" tree..." % self.tree_name)
            
        print("Running \"%s\" tree..." % self.tree_name)

        self.tick_tock(period_ms=(1000 / hz))
        return True

    def visualise(self, image_type='svg'):
        if image_type not in ['svg', 'png']:
            raise ValueError(
                "ERROR: only 'svg' & 'png' image types are supported")
        _validate_tmp()
        filename = os.path.join(
            _TMP_LOCATION,
            datetime.datetime.now().strftime('%Y%m%d_%H%M%S'))
        pt.display.render_dot_tree(self.root, name=filename)
        subprocess.call(["xdg-open", filename + '.' + image_type])

    @staticmethod
    def get():
      return BehaviourTree._current
