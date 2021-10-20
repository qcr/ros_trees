import random

from .. import data_management as dm
from ..leaves import Leaf


class PushItem(Leaf):

    def __init__(self, name='Push item', key=None, *args, **kwargs):
        if not key:
            raise ValueError('Missing required field: key')

        super(PushItem, self).__init__(name=name,
                                       save_fn=self.save_fn,
                                       save=True,
                                       *args,
                                       **kwargs)
        self.key = key

    def eval_fn(self, value):
        return True

    def save_fn(self, value):
        entries = dm.get_value(self.key)
        if not entries:
            entries = []

        entries.append(self.loaded_data)
        dm.set_value(self.key, entries)

        self._default_save_fn(self.loaded_data)

        return self.loaded_data


class PopItem(Leaf):

    def __init__(self, name='Pop item', key=None, save=True, *args, **kwargs):
        if not key:
            raise ValueError('Missing required field: key')

        super(PopItem, self).__init__(name=name,
                                      result_fn=self.result_fn,
                                      save=save,
                                      *args,
                                      **kwargs)
        self.key = key

    def result_fn(self):
        entries = dm.get_value(self.key)
        if not entries:
            return None

        item = entries.pop(0)
        dm.set_value(self.key, entries)

        return item


class PeekItem(Leaf):

    def __init__(self, name='Peek item', key=None, save=True, *args, **kwargs):
        if not key:
            raise ValueError('Missing required field: key')

        super(PeekItem, self).__init__(name=name,
                                       result_fn=self.result_fn,
                                       save=save,
                                       *args,
                                       **kwargs)
        self.key = key

    def result_fn(self):
        entries = dm.get_value(self.key)
        if not entries:
            return None

        item = entries[0]
        dm.set_value(self.key, entries)

        return item


class ClearItems(Leaf):

    def __init__(self, name='Clear Items', key=None, *args, **kwargs):
        if not key:
            raise ValueError('Missing required field: key')

        super(ClearItems, self).__init__(name=name,
                                         result_fn=self.result_fn,
                                         *args,
                                         **kwargs)
        self.key = key

    def result_fn(self):
        dm.set_value(self.key, [])
        return True


class ChooseRandom(Leaf):

    def __init__(self, name='Select Random', items=[], *args, **kwargs):
        super(ChooseRandom, self).__init__(name=name,
                                           load_fn=self.load_fn,
                                           save=True,
                                           *args,
                                           **kwargs)
        self.items = items

    def load_fn(self):
        return random.choice(self.items)

