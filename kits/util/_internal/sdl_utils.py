

def SDL_JoystickGetGUIDString(guid):
  """See https://github.com/marcusva/py-sdl2/issues/75 for explanation"""
  s = ''
  for g in guid.data:
    s += "{:x}".format(g >> 4)
    s += "{:x}".format(g & 0x0F)
  return s