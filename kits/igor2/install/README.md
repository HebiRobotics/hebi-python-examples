# Igor Demo Service

To run the Igor demo on startup, you have the option to add the demo as a service on Linux.
**Please note that this tutorial assumes that your machine's username is `hebi`!**

## Requirements

This is currently only supported on GNU/Linux systems using Systemd.
Consequently, the following distros are guaranteed to work:

* Ubuntu (16.04+) and all of its "flavors" (_e.g._, Kubuntu, Xubuntu, etc)
  * **Ubuntu 14.04 does not use systemd and thus is not compatible.**
* Fedora and its derivatives (Red Hat, CentOS)
* Debian
* Arch Linux

This list is _not_ exhaustive, but if you are using a distro not mentioned above, you probably know what you are doing.

**Note:** The service assumes you have Python 3. Python 2 is nearly EOL and should be avoided. You must modify the shell scripts to use Python 2 if you must support a legacy environment.

## Installing (Easy Guide)

To automate the steps shown in the manual guide below, you can run the `service_install` shell script in this folder
as root. If you would prefer to manually install the service yourself, then you can do it step-by step as shown below.

## Installing (Manual)

### Setup

This tutorial (and corresponding scripts) assume that you have the `hebi-python-examples` (the repo you are viewing right now)
cloned/copied to the directory `/home/hebi/hebi-python-examples`. You can do this by executing in a terminal:

```sh
# 1: Clone repository to home folder
cd /home/hebi
git clone https://www.github.com/HebiRobotics/hebi-python-examples
```

*Note:* All of the following shell script excerpts assume you have changed directories to `/home/hebi/hebi-python-examples/kits/igor2/install`.

### Systemd Configuration

The `hebi-demo.service` file is used to describe the service to the `init` system (systemd). You can copy it to any
of the unit file directories systemd uses to find services. For example, you can copy it in a terminal:

```sh
# 2: Copy service file
cp hebi-demo.service /usr/lib/systemd/system/
```

*Note:* In general, you need to be the superuser (root) in order to write any files to these directories.

----

To have systemd recognize the newly installed service file, you need to run (as root) the following command:

```sh
# 3: Reload all service files (scanning for new ones as well)
systemctl daemon-reload
```

----

To check that your computer has found the newly installed `hebi-demo` service, you can run the following command (does not need to be root):

```sh
# 4: Check that systemd found the service
systemctl status hebi-demo.service
```

If the above command fails with a message like:

```
Unit hebi-demo.service could not be found.
```

Then make sure that you have installed the `hebi-demo.service` file to the correct directory.
Otherwise, if systemd is aware of the new service, it will say that it is not running and is disabled.

----

At this point, you need to make sure to copy the `launch_demo` and `hebi_demo_wait_for_connected` scripts to the home directory:

```sh
# 5: Copy shell scripts to home directory - also make sure to set executable bits
cp hebi_demo_wait_for_connected /home/hebi && chmod +x /home/hebi/hebi_demo_wait_for_connected
cp launch_demo /home/hebi && chmod +x /home/hebi/launch_demo
```

You can now start or enable the demo.

----

In order to enable the demo on computer startup, you must _enable_ the service. This is different than _starting_ (_i.e._, running) the service.
Running as root, execute the following:

```sh
# 6: Enable service to run on startup
systemctl enable hebi-demo.service
```

## Troubleshooting

### Reading program output

The demo's standard output (`stdout`) is redirected to systemd's syslog. This means that you can view any output (whether it be from `stdout` or `stderr`) by executing in a shell (no need for root):

```sh
journalctl -u hebi-demo.service
```

To jump to the end of the log, you can also try:

```sh
journalctl -u hebi-demo.service -e
```

This is very useful when debugging any python exceptions not caught, or messages printed out from python.

### Demo loops endlessly in `hebi_demo_wait_for_connected` script

If your `launch_demo` script loops endlessly in the wait for connected script, you should see similar output from the syslog:

```sh
journalctl -u hebi-demo.service -e
>>> Mar 30 10:02:24 localhost.localdomain hebi-demo.service[NUMBER]: Waiting for network interface 'eno1' to come up...
>>> Mar 30 10:02:27 localhost.localdomain hebi-demo.service[NUMBER]: Waiting for network interface 'eno1' to come up...
>>> Mar 30 10:02:30 localhost.localdomain hebi-demo.service[NUMBER]: Waiting for network interface 'eno1' to come up...
>>> Mar 30 10:02:33 localhost.localdomain hebi-demo.service[NUMBER]: Waiting for network interface 'eno1' to come up...
>>> Mar 30 10:02:36 localhost.localdomain hebi-demo.service[NUMBER]: Waiting for network interface 'eno1' to come up...
>>> Mar 30 10:02:39 localhost.localdomain hebi-demo.service[NUMBER]: Waiting for network interface 'eno1' to come up...
```

#### 1) Checking for connectivity

The first thing to check is if the router is on and giving the modules and your machine an IP address. From the command line,
you can see if your machine has a network interface with an IP by executing in a terminal:

```sh
ip a
```

You should search for an IP address, which may work with the command:

```sh
ip a | grep -o -e 'inet[[:space:]]\{1,\}[[:digit:]]\{1,3\}.[[:digit:]]\{1,3\}.[[:digit:]]\{1,3\}.[[:digit:]]\{1,3\}' | awk '{ print $2 }'
```

an example output may be:

```
127.0.0.1
192.168.1.133
192.168.122.1
```

If you have an IP address in addition to the loopback (127.0.0.1), then at least one of your network devices is configured properly.
Otherwise, you should check if the network cable from your machine to the router connected to the modules is disconnected.
If you are sure your machine is physically connected to the router, try restarting the router or troubleshooting starting there.

#### 2) Checking the network interface name

The next thing to check is what the name of your ethernet connection is on your machine. Linux will sometimes
rename the interface in a way which is not entirely predictable. The `hebi_demo_wait_for_connected` assumes
the name of the ethernet device is `eno1`.

To view all of your network interfaces on your machine, execute the following in a terminal:

```sh
ip a
```

To view just the actual interface names, execute:

```sh
ip a show | grep -e "^[[:digit:]]\{1,\}" | awk '{print $2}' | tr ':' ' '
```

An example output for the above command on a machine with 2 ethernet ports and a wireless card is:

```sh
lo 
enp6s0 
eno1 
wlp5s0
```

If we were using only the `enp6s0` interface, we would change the line in `hebi_demo_wait_for_connected` to:

```sh
# Change this to the network interface connected to the subnet containing the modules of your robot.
DEPLOY_IFACE=enp6s0
```