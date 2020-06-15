#!/usr/bin/env python3

import hebi

folder = './logs/'
name = 'exampleLog.hebilog'

log = hebi.util.load_log('{}/{}'.format(folder, name))

# Plot using some handy helper functions
hebi.util.plot_logs(log, 'position', figure_spec=101)

log = hebi.util.load_log('{}/{}'.format(folder, name))
hebi.util.plot_logs(log, 'velocity', figure_spec=102)

log = hebi.util.load_log('{}/{}'.format(folder, name))
hebi.util.plot_logs(log, 'effort', figure_spec=103)
