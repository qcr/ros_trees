from __future__ import print_function
import sys
import select

from ros_trees.leaves import Leaf

try:
    input = raw_input
except NameError:
    pass


class Print(Leaf):
    def __init__(self, name=None, format_string=None, *args, **kwargs):
        super(Print, self).__init__(name if name else 'Print',
                                    result_fn=self._result_fn,
                                    *args,
                                    **kwargs)
        self.format_string = format_string

    def _result_fn(self):
        if self.format_string:
            print(self.format_string % self.loaded_data)
        else:
            print(self.loaded_data)
        return self.loaded_data

class Read(Leaf):
    def __init__(self, name='Read', prompt='> ', timeout=None, strip=False, save=True, *args, **kwargs):
        super(Read, self).__init__(name,
                                    result_fn=self._result_fn,
                                    save=save,
                                    *args,
                                    **kwargs)
        self.timeout = timeout
        self.prompt = prompt
        self.strip = strip

    def _result_fn(self):
        print(self.prompt, end='')
        sys.stdout.flush()

        if len(select.select([sys.stdin], [], [], self.timeout)[0]) > 0:
          text = sys.stdin.readline()
          return text.strip() if self.strip else text
        
        print()
        return False

class SelectItem(Leaf):
    # NOTE: this is BLOCKING & should ONLY BE USED for debugging purposes
    _DEFAULT_SELECT_TEXT = "Please select an item"

    def __init__(self, select_text=None, *args, **kwargs):
        super(SelectItem, self).__init__("Select Item",
                                         result_fn=self._result_fn,
                                         save=True,
                                         *args,
                                         **kwargs)
        self.select_text = (SelectItem._DEFAULT_SELECT_TEXT
                            if select_text is None else select_text)

    def _result_fn(self):
        items = self.loaded_data
        if (not isinstance(items, list) or len(items) == 0 or
                not any([isinstance(i, str) for i in items])):
            ValueError(
                "SelectItem leaf expects a list of strings as loaded_data. "
                "It has been provided with: %s" % (items))

        item = None
        while (item is None or item not in items):
            item = input("%s %s: " % (self.select_text, items))

        return item
