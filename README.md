yausbir_lirc
============

yausbir_lirc is a simple LIRC UDP daemon which can be used to get the yaUsbIR receiver module connected to an unpatched LIRC version.

yausbir_lirc has been initially written by "uwe67" and abandoned in favor of the LIRC driver patch which also features sending IR codes.

As I prefer to use an unpatched LIRC, and wanted to learn something about the basics of infrared code decoding, I decided to backport all the fixes and improvements, that only landed in the driver patch, to the UDP daemon.

Setup
-----

  make
  make install

Now get sure that your LIRC daemon gets started with the "UDP" driver

  lircd -H udp

Finally start the yausbir_lirc daemon

  yausbir_lirc

If your distribution uses systemd, then "make install" installs a service file for you. In this case you may just do the following:

  systemctl enable yausbir_lirc
  systemctl start yausbir_lirc
