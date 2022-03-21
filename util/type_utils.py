import hebi


def assert_callable(val, what='function'):
  """
  :param val:
  :param what:
  """
  if not hasattr(val, '__call__'):
    raise TypeError('{0} is not a callable object (does not have __call__)'.format(what))


def assert_type(val, type_, what='input'):
  """
  :param val:
  :param type_:
  :param what:
  """
  if type(val) != type_:
    raise TypeError('{0} must be of type {1}'.format(what, type_.__name__))


def assert_range(val, infimum, supremum, what='index', l_inclusive=True, u_inclusive=False):
  """
  :param val:
  :param infimum:
  :param supremum:
  :param what:
  :param l_inclusive:
  :param u_inclusive:
  """
  if l_inclusive:
    l_str = '['
    in_range = val >= infimum
  else:  # exclusive
    l_str = '('
    in_range = val > infimum

  if u_inclusive:
    u_str = ']'
    in_range = in_range and val <= supremum
  else:  # exclusive
    u_str = ')'
    in_range = in_range and val < supremum

  if not in_range:
    raise IndexError('{0} ({5}) out of range {1}{2},{3}{4}'.format(
        what, l_str, infimum, supremum, u_str, val))


def assert_prange(val, supremum, what='index'):
  """
  :param val:
  :param supremum:
  :param what:
  """
  assert_range(val, 0, supremum, what=what)
