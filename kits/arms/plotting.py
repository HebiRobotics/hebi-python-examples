from matplotlib import pyplot as plt

def draw_plots(hebi_log):
    
    # Plot tracking / error from the joints in the arm.
    time = []
    position = []
    velocity = []
    effort = []
    # iterate through log
    for entry in hebi_log.feedback_iterate:
        time.append(entry.transmit_time)
        position.append(entry.position)
        velocity.append(entry.velocity)
        effort.append(entry.effort)

    # Offline Visualization
    # Plot the logged position feedback
    plt.figure(101)
    plt.plot(time, position)
    plt.title('Position')
    plt.xlabel('time (sec)')
    plt.ylabel('position (rad)')
    plt.grid(True)

    # Plot the logged velocity feedback
    plt.figure(102)
    plt.plot(time, velocity)
    plt.title('Velocity')
    plt.xlabel('time (sec)')
    plt.ylabel('velocity (rad/sec)')
    plt.grid(True)

    # Plot the logged effort feedback
    plt.figure(103)
    plt.plot(time, effort)
    plt.title('Effort')
    plt.xlabel('time (sec)')
    plt.ylabel('effort (N*m)')
    plt.grid(True)

    plt.show()