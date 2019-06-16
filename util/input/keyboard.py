import sys


# ------------------------------------------------------------------------------
# getch API
# ------------------------------------------------------------------------------


class _Getch:
  def __init__(self):
    try:
      self.impl = _GetchWindows()
    except ImportError:
      try:
        self.impl = _GetchMacCarbon()
      except(AttributeError, ImportError):
        self.impl = _GetchUnix()

  def getch_blocking(self): return self.impl.getch_blocking()
  def getch_non_blocking(self): return self.impl.getch_non_blocking()


class _GetchUnix:
  def __init__(self):
    import select
    from termios import tcgetattr, tcsetattr, TCSADRAIN
    from tty import setraw

    self._kbhit = lambda: select.select([sys.stdin], [], [], 0)[0] != []
    self._getch = lambda: sys.stdin.read(1)


    def push_terminal_unbuffered(term_attrs):
      term_attrs[3] = (term_attrs[3] & ~termios.ICANON & ~termios.ECHO)
      termios.tcsetattr(sys.stdin.fileno(), termios.TCSAFLUSH, term_attrs)


    self._get_terminal_settings = lambda: tcgetattr(sys.stdin.fileno())
    self._push_terminal_unbuffered = push_terminal_unbuffered
    self._pop_terminal_unbuffered = lambda term_attrs: termios.tcsetattr(sys.stdin.fileno(), termios.TCSAFLUSH, term_attrs)
    self._push_terminal_raw = lambda: setraw(sys.stdin.fileno())
    self._pop_terminal_raw = lambda settings: tcsetattr(sys.stdin.fileno(), TCSADRAIN, settings)

  def getch_blocking(self):
    old_settings = self._get_terminal_settings()
    ch = ''
    try:
      self._push_terminal_raw()
      ch = self._getch()
    finally:
      self._pop_terminal_raw(old_settings)
    return ch

  def getch_non_blocking(self):
    old_settings = self._get_terminal_settings()
    # Call again so we don't modify the previous settings
    self._push_terminal_unbuffered(self._get_terminal_settings())
    ch = ''
    try:
      if self._kbhit():
        ch = self._getch()
    finally:
      self._pop_terminal_unbuffered(old_settings)
    return ch


class _GetchWindows:
  def __init__(self):
    import msvcrt
    if sys.version_info[0] == 3:
      def get_char():
        ch = msvcrt.getch()
        if type(ch) is bytes:
          return ch.decode('utf8')
        return ch
      self._getch = get_char
    else:
      self._getch = msvcrt.getch
    self._kbhit = msvcrt.kbhit

  def getch_blocking(self):
    return self._getch()

  def getch_non_blocking(self):
    if self._kbhit():
      return self._getch()
    return ''


class _GetchMacCarbon:
  """
  A function which returns the current ASCII key that is down;
  if no ASCII key is down, the null string is returned.  The
  page http://www.mactech.com/macintosh-c/chap02-1.html was
  very helpful in figuring out how to do this.
  """
  def __init__(self):
    import Carbon
    # This doesn't exist in Unix, so this will raise an exception.
    evt_module = Carbon.Evt
    self._has_event_avail = lambda: Carbon.Evt.EventAvail(0x0008)[0] != 0
    self._get_next_event = lambda: chr(Carbon.Evt.GetNextEvent(0x0008)[1][1] & 0x000000FF)

  def getch_blocking(self):
    if self._has_event_avail():
      # TODO: This isn't really blocking...
      return self._get_next_event()
    return ''

  def getch_non_blocking(self):
    if self._has_event_avail():
      return self._get_next_event()
    return ''


# Singleton
_getch = _Getch()


def getch(blocking=True):
  if blocking:
    return _getch.getch_blocking()
  return _getch.getch_non_blocking()


# ------------------------------------------------------------------------------
# Keyboard Events API
# ------------------------------------------------------------------------------


import sdl2


class _Keyboard:
  def __init__(self):
    kbd_event_map = dict()
    self.__kbd_event_map = kbd_event_map

  def _on_key_event(self, ts, state, repeat, keysym):
    keystr = keysym.sym
    if keystr in self.__kbd_event_map:
      handlers = self.__kbd_event_map[keystr]
      for handler in handlers:
        handler(ts, state == sdl2.SDL_PRESSED, repeat)

  def add_key_event_handler(self, keystr, handler):
    if not callable(handler):
      # TODO: use util.assert_callable
      raise TypeError

    if type(keystr) is not int:
      keystr = ord(keystr)
    else:
      # TODO: Make sure keycode is valid...
      pass

    if keystr not in self.__kbd_event_map:
      self.__kbd_event_map[keystr] = [handler]
    else:
      self.__kbd_event_map[keystr].append(handler)


_kbd_instance = _Keyboard()


class _EscNoop:
  def __init__(self): pass
  def clear(self): return False
  def event_handler(self, ts, pressed, repeat): pass
  @property
  def has_been_pressed(self): return False
    

__esc_listener = _EscNoop()


def _listen_for_esc():
  if __esc_listener is not None:
    # already listening - return
    return

  global __esc_listener


  class EscListener:
    def __init__(self):
      self._has_been_pressed = False
      _kbd_instance.add_key_event_handler(sdl2.SDLK_ESCAPE, self.event_handler)

    def clear(self):
      ret = self._has_been_pressed
      self._has_been_pressed = False
      return ret

    def event_handler(self, ts, pressed, repeat):
      self._has_been_pressed = pressed or self._has_been_pressed

    @property
    def has_been_pressed(self):
      return self._has_been_pressed


  __esc_listener = EscListener()


def _clear_esc_state():
  return __esc_listener.clear()


def _has_esc_been_pressed():
  return __esc_listener.has_been_pressed

