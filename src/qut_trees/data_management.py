import py_trees as pt

LAST_RESULT_KEY = 'last_result'
_LAST_RESULT_PREFIX = LAST_RESULT_KEY + '_'


def _get_previous_leaf(leaf):
    # The "previous leaf" in the tree is the previous leaf in the TREE
    # HIERARCHY (i.e. it is NOT the last leaf executed, but the previous if
    # going "backwards" through the tree)

    # Search up the tree until we get something where we are not the first item
    # in the child list
    idx = 0
    child = leaf
    while idx == 0 and child.parent is not None:
        idx = child.parent.children.index(child)
        if idx == 0:
            child = child.parent
    previous_leaf = (None if idx == 0 else child.parent.children[idx - 1])

    # We have found a leaf, or branch that our previous leaf resides in. If it
    # is a branch we now have to dig back down & find the end leaf in this part
    # of the hieararchy
    while previous_leaf is not None and previous_leaf.children:
        previous_leaf = previous_leaf.children[-1]
    return previous_leaf


def _get_last_result_key(leaf):
    # Returns the last result key associated with a given leaf
    return _LAST_RESULT_PREFIX + str(id(leaf))


def get_last_value(leaf=None):
    # Generally you want to pass in a leaf, & we will interpret "last result"
    # as the result of the previous leaf in the tree hierarchy. Otherwise we
    # will give you the result written by the last leaf EXECUTED in the tree
    # which could be ANY OF THEM
    if leaf is None:
        return get_value(LAST_RESULT_KEY)
    else:
        result = None
        leaf_old = None
        while leaf is not None and result is None:
            leaf_old = leaf
            leaf = _get_previous_leaf(leaf_old)
            result = get_value(_get_last_result_key(leaf))
        return result


def get_last_value_field(leaf=None, num=0):
    return to_fields_list(get_last_value(leaf))[num]


def get_value_field(key_name, num=0):
    return to_fields_list(get_value(key_name))[num]


def get_value(key_name):
    # Returns None if there is no key matching key_name
    return pt.Blackboard().get(
        LAST_RESULT_KEY if key_name is None else key_name)


def set_last_value(leaf, value):
    # Support both last EXECUTED and previous leaf in tree by setting both
    # NOTE: we tolerate leaf == None, but should be avoided
    set_value(LAST_RESULT_KEY, value)
    if leaf is not None:
        set_value(_get_last_result_key(leaf), value)


def set_value(key_name, value):
    pt.Blackboard().set(key_name, value)


def to_fields_list(obj):
    # Returns a list of the fields in an object, where the order corresponds to
    # their order in the object (confirmed to work for ROS Service Requests &
    # Responses, as well as ROS Action Server Goals & Results)
    return [getattr(obj, a) for a in type(obj).__dict__['__slots__']]
