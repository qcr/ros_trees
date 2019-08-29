import py_trees as pt

from . import data_management as dm
from .debugging import DebugMode as DM


class Leaf(pt.behaviour.Behaviour):

    def __init__(self,
                 name,
                 load=True,
                 load_value=None,
                 load_key=None,
                 load_fn=None,
                 result_fn=None,
                 eval_fn=None,
                 save=False,
                 save_value=None,
                 save_key=None,
                 save_fn=None,
                 debug=DM.OFF):
        super(Leaf, self).__init__(name)

        # Setup all values (sanitising to remove nonsensical combinations)
        self.load = (load if load_value is None and load_key is None else True)
        self.load_value = load_value
        self.load_key = load_key if self.load_value is None else None
        self.load_fn = self._default_load_fn if load_fn is None else load_fn
        self.loaded_data = None

        self.result_fn = (self._default_result_fn
                          if result_fn is None else result_fn)
        self.eval_fn = self._default_eval_fn if eval_fn is None else eval_fn

        self.save = (save if save_value is None and save_key is None else True)
        self.save_value = save_value
        self.save_key = save_key
        self.save_fn = self._default_save_fn if save_fn is None else save_fn

        self.debug = debug
        self._debug_key_received = False

    def _default_eval_fn(self, value):
        if isinstance(value, list):
            first_bool = next((i for i in value if isinstance(i, bool)), None)
            return bool(value[0]) if first_bool is None else first_bool
        else:
            return bool(value)

    def _default_load_fn(self):
        if self.load_value is None:
            return (dm.get_last_value(self)
                    if self.load_key is None else dm.get_value(self.load_key))
        else:
            return self.load_value

    def _default_result_fn(self):
        return self.loaded_data

    def _default_save_fn(self, value):
        if self.save_key is not None:
            dm.set_value(self.save_key, value)
        else:
            dm.set_last_value(self, value)

    def _extra_initialise(self):
        pass

    def _extra_setup(self, timeout):
        return True

    def _extra_terminate(self, new_status):
        pass

    def _extra_update(self):
        return None

    def _is_leaf_done(self):
        return True

    def _requirements_str(self):
        return None

    def initialise(self):
        # Handle logging & debugging
        self.logger.debug("%s.initialise()" % self.__class__.__name__)
        super(Leaf, self).initialise()
        self._debug_key_received = False
        if self.debug != DM.OFF:
            return

        # Call any extra initialisation steps & the load function
        self._extra_initialise()
        self.loaded_data = self.load_fn() if self.load else None

    def setup(self, timeout):
        # Handle logging & debugging
        self.logger.debug("%s.setup()" % self.__class__.__name__)
        super(Leaf, self).setup(timeout)

        # Call any extra setup steps (if not in debugging mode)
        return (True if self.debug != DM.OFF else self._extra_setup(timeout))

    def terminate(self, new_status):
        # Handle logging
        self.logger.debug("%s.terminate()" % self.__class__.__name__)
        super(Leaf, self).terminate(new_status)

        # Call any extra termination steps (if not in debugging mode)
        if self.debug == DM.OFF:
            self._extra_terminate(new_status)

    def update(self):
        # Handle logging and debugging early exits
        self.logger.debug("%s.update()" % self.__class__.__name__)
        if ('INSTANT' in self.debug.name or
            ('INPUT' in self.debug.name and self._debug_key_received)):
            self.logger.info("%s.update(): Skipping \"%s\"" %
                             (self.__class__.__name__, self.name))
            return (pt.Status.SUCCESS
                    if 'SUCCESS' in self.debug.name else pt.Status.FAILURE)
        elif ('INPUT' in self.debug.name):
            return pt.Status.RUNNING

        # Call any extra parts of the update step pylint:
        ret = self._extra_update()  # pylint: disable=assignment-from-none
        if ret != None:
            return ret

        # Either return a running state, or evaluate & return the final result
        if self._is_leaf_done():
            # Get the result of the update step (incorporating any extra update
            # processes)
            result = (self.result_fn()
                      if self.save_value is None else self.save_value)

            # Save the result if requested
            if self.save:
                self.save_fn(result)

            # Evaluate the result and act accordingly
            ret = (pt.Status.SUCCESS
                   if self.eval_fn(result) else pt.Status.FAILURE)
            self.feedback_message = "finished with %s" % ret.name
            return ret
        else:
            self.feedback_message = "running..."
            return pt.Status.RUNNING
