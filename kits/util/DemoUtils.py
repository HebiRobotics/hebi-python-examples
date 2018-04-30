from hebi import GroupCommand
from time import sleep


def retry_on_error(func, on_error_func=None, sleep_time=0.1):
  while True:
    try:
      ret = func()
      return ret
    except Exception as e:
      if on_error_func != None:
        on_error_func()
      sleep(sleep_time)


def send_gains(group, gains_file):
  command = GroupCommand(group.size)
  command.read_gains(gains_file)

  # Disable feedback frequency to reduce messages
  prev_freq = group.feedback_frequency
  group.feedback_frequency = 0

  sleep(0.25)
  for i in range(5):
    group.send_command(command)
    sleep(0.1)

  group.feedback_frequency = prev_freq