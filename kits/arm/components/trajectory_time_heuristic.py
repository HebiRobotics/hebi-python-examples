

def get_times(positions, velocities, accelerations):
  times = np.empty(positions.shape[1])
  for i in range(positions.shape[1]):
    times[i] = i * 2
  return times
