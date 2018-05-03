import threading
import cProfile


def dump_c_profiling():
  try:
    from hebi._internal.raw import _dump_c_api_stats
    _dump_c_api_stats()
  except Exception as e:
    print('Could not dump C profiling statistics: {0}'.format(e))


class ProfiledThread(threading.Thread):
  def run(self):
    profiler = cProfile.Profile()
    try:
      return profiler.runcall(threading.Thread.run, self)
    finally:
      dump_c_profiling()
      profiler.dump_stats('{0}-{1}.profile'.format(self.name, self.ident))

