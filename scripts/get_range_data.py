#!/usr/bin/env python
"""
Updated by Evagoras Makridis (evagoras.makridis@gmail.com)
4 July 2019 11:24

The Pozyx ready to localize tutorial (c) Pozyx Labs
Please read the tutorial that accompanies this sketch:
https://www.pozyx.io/Documentation/Tutorials/ready_to_localize/Python

This tutorial requires at least the contents of the Pozyx Ready to Localize kit. It demonstrates the positioning capabilities
of the Pozyx device both locally and remotely. Follow the steps to correctly set up your environment in the link, change the
parameters and upload this sketch. Watch the coordinates change as you move your device around!
"""
from time import sleep
from pypozyx import *
from pythonosc.udp_client import SimpleUDPClient

global f
class ReadyToRange(object):
    def __init__(self, pozyx, destination_id, range_step_mm=1000, protocol=POZYX_RANGE_PROTOCOL_PRECISION, remote_id=None):
      self.pozyx = pozyx
      self.destination_id = destination_id
      self.range_step_mm = range_step_mm
      self.remote_id = remote_id
      self.protocol = protocol

    def setup(self):
      print("------------POZYX RANGING -------------")
      print("NOTES: ")
      print("- Change the parameters:")
      print("\tdestination_id(target_device)")
      print("\trange_step(mm)")
      print()
      print("- Approach target device to see range")
      print()
      if self.remote_id is None:
        for device_id in [self.remote_id, self.destination_id]:
          self.pozyx.printDeviceInfo(device_id)
      else:
         for device_id in [None, self.remote_id, self.destination_id]:
          self.pozyx.printDeviceInfo(device_id)
      print()
      print("Start ranging:")
      self.pozyx.setRangingProtocol(self.protocol, self.remote_id)

    def loop(self):
      device_range = DeviceRange()
      status = self.pozyx.doRanging(self.destination_id, device_range, self.remote_id)
      if status == POZYX_SUCCESS:
        print(device_range)
        f.write(str(device_range.timestamp)+","+str(device_range.RSS)+","+str(device_range.distance))
        f.write(" \n")
      else:
        error_code = SingleRegister()
        status = self.pozyx.getErrorCode(error_code)
        if status == POZYX_SUCCESS:
          print("ERROR Ranging, local %s" % self.pozyx.getErrorMessage(error_code))
        else:
          print("ERROR Ranging, couldn't retrieve local error")    


if __name__ == "__main__":
    # shortcut to not have to find out the port yourself
    serial_port = get_first_pozyx_serial_port()
    if serial_port is None:
        print("No Pozyx connected. Check your USB cable or your driver!")
        quit()

    remote_id = 0x6069	# remote device network ID
    remote = False	# whether to use a remote device
    if not remote:
        remote_id = None

    destination_id = 0x6e1a	# Change accordingly with the anchor id (written on it) you want to range
    range_step_mm = 1000
    ranging_protocol = POZYX_RANGE_PROTOCOL_PRECISION
    pozyx = PozyxSerial(serial_port)
    r = ReadyToRange(pozyx,destination_id, range_step_mm, ranging_protocol, remote_id)
    r.setup()
    f=open("ranging_data/ranging_"+hex(destination_id)+".txt", "w+")

    while True:
        r.loop()
