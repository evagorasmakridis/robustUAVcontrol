#!/usr/bin/env python
"""
The Pozyx ready to localize tutorial (c) Pozyx Labs
Please read the tutorial that accompanies this sketch:
https://www.pozyx.io/Documentation/Tutorials/ready_to_localize/Python

This tutorial requires at least the contents of the Pozyx Ready to Localize kit. It demonstrates the positioning capabilities
of the Pozyx device both locally and remotely. Follow the steps to correctly set up your environment in the link, change the
parameters and upload this sketch. Watch the coordinates change as you move your device around!
"""
import rospy
from pypozyx import (POZYX_POS_ALG_UWB_ONLY,POZYX_3D,Coordinates,POZYX_SUCCESS,PozyxConstants,version,DeviceCoordinates,PozyxSerial,get_first_pozyx_serial_port,SingleRegister,DeviceList,PozyxRegisters)
from time import sleep
from dji_sdk_demo.msg import Pos
from pythonosc.osc_message_builder import OscMessageBuilder
from pythonosc.udp_client import SimpleUDPClient

class ReadyToLocalize(object):
    """Continuously calls the Pozyx positioning function and prints its position."""
  
    def __init__(self, pozyx, osc_udp_client, anchors, algorithm=POZYX_POS_ALG_UWB_ONLY, dimension=POZYX_3D, height=1000, filter_type=PozyxConstants.FILTER_TYPE_MOVING_AVERAGE, filter_strength=1, remote_id=None):
        self.pozyx = pozyx
        self.osc_udp_client = osc_udp_client
       
        self.anchors = anchors
        self.algorithm = algorithm
        self.dimension = dimension
        self.height = height
        self.remote_id = remote_id
        self.filter_type = filter_type
        self.filter_strength = filter_strength
        
    def setup(self):
        """Sets up the Pozyx for positioning by calibrating its anchor list."""
        print("------------POZYX POSITIONING V1.1 -------------")
        print("NOTES: ")
        print("- No parameters required.")
        print()
        print("- System will auto start configuration")
        print()
        print("- System will auto start positioning")
        print()
        if self.remote_id is None:
          self.pozyx.printDeviceInfo(self.remote_id)
        else:
          for device_id in [None, self.remote_id]:
            self.pozyx.printDeviceInfo(device_id)
        print()
        print("------------POZYX POSITIONING V1.1 --------------")
        print()

        self.setAnchorsManual(save_to_flash=False)
        self.printPublishConfigurationResult()
        self.pozyx.setPositionFilter(self.filter_type, self.filter_strength, self.remote_id)

    def loop(self):
        """Performs positioning"""
        position = Coordinates()
        status = self.pozyx.doPositioning(position, self.dimension, self.height, self.algorithm, remote_id=self.remote_id)
        if status == POZYX_SUCCESS:
          curr_pos=Pos()
          curr_pos.x = position.x/1000.0   
          curr_pos.y = position.y/1000.0   
          curr_pos.z = position.z/1000.0   
          pub.publish(curr_pos)
        else:
          self.printPublishErrorCode("positioning")          

    def setAnchorsManual(self, save_to_flash=False):
        """Adds the manually measured anchors to the Pozyx's device list one for one."""
        status = self.pozyx.clearDevices(remote_id=self.remote_id)
        for anchor in self.anchors:
            status &= self.pozyx.addDevice(anchor, remote_id=self.remote_id)
        if len(self.anchors) > 4:
            status &= self.pozyx.setSelectionOfAnchors(PozyxConstants.ANCHOR_SELECT_AUTO, len(self.anchors),remote_id=self.remote_id)

        if save_to_flash:
            self.pozyx.saveAnchorIds(remote_id=self.remote_id)
            self.pozyx.saveRegisters([PozyxRegisters.POSITIONING_NUMBER_OF_ANCHORS], remote_id=self.remote_id)
        return status

    def printPublishConfigurationResult(self):
        """Prints and potentially publishes the anchor configuration result in a human-readable way."""
        list_size = SingleRegister()

        status = self.pozyx.getDeviceListSize(list_size, self.remote_id)
        print("List size: {0}".format(list_size[0]))
        if list_size[0] != len(self.anchors):
            self.printPublishErrorCode("configuration")
            return
        device_list = DeviceList(list_size=list_size[0])
        status = self.pozyx.getDeviceIds(device_list, self.remote_id)
        print("Calibration result:")
        print("Anchors found: {0}".format(list_size[0]))
        print("Anchor IDs: ", device_list)

        for i in range(list_size[0]):
            anchor_coordinates = Coordinates()
            status = self.pozyx.getDeviceCoordinates(
                device_list[i], anchor_coordinates, self.remote_id)
            print("ANCHOR, 0x%0.4x, %s" % (device_list[i], str(anchor_coordinates)))
            if self.osc_udp_client is not None:
                self.osc_udp_client.send_message(
                    "/anchor", [device_list[i], int(anchor_coordinates.x), int(anchor_coordinates.y), int(anchor_coordinates.z)])
                sleep(0.025)

    def printPublishAnchorConfiguration(self):
        """Prints and potentially publishes the anchor configuration"""
        for anchor in self.anchors:
            print("ANCHOR,0x%0.4x,%s" % (anchor.network_id, str(anchor.coordinates)))
            if self.osc_udp_client is not None:
                self.osc_udp_client.send_message(
                    "/anchor", [anchor.network_id, int(anchor_coordinates.x), int(anchor_coordinates.y), int(anchor_coordinates.z)])
                sleep(0.025)

    def printPublishErrorCode(self, operation):
        error_code = SingleRegister()
        network_id = self.remote_id
        if network_id is None:
            self.pozyx.getErrorCode(error_code)
            print("LOCAL ERROR %s, %s" %(operation, self.pozyx.getErrorMessage(error_code)))
            if self.osc_udp_client is not None:
                self.osc_udp_client.send_message("/error", [operation, 0, error_code[0]])
            return
        status = self.pozyx.getErrorCode(error_code, self.remote_id)
        if status == POZYX_SUCCESS:
            print("ERROR %s on ID %s, %s" %(operation, "0x%0.4x" % network_id, self.pozyx.getErrorMessage(error_code))) 
            if self.osc_udp_client is not None:
                self.osc_udp_client.send_message("/error", [operation, network_id, error_code[0]])
        else:
            self.pozyx.getErrorCode(error_code)
            print("ERROR %s, couldn't retrueve remote error code, LOCAL ERROR %s" %(operation, self.pozyx.getErrorMessage(error_code))) 
            if self.osc_udp_client is not None:
                self.osc_udp_client.send_message("/error", [operation, 0, -1])
       


if __name__ == "__main__":
    # shortcut to not have to find out the port yourself
    serial_port = get_first_pozyx_serial_port()
    if serial_port is None:
        print("No Pozyx connected. Check your USB cable or your driver!")
        quit()

    remote_id = 0x6e5e                 # remote device network ID
    remote = False                   # whether to use a remote device
    if not remote:
        remote_id = None

    use_processing = True             # enable to send position data through OSC
    ip = "127.0.0.1"                   # IP for the OSC UDP
    network_port = 8888                # network port for the OSC UDP
   
    osc_udp_client = None
    if use_processing:
        osc_udp_client = SimpleUDPClient(ip, network_port)

    # necessary data for calibration, change the IDs and coordinates yourself
    #anchors = [DeviceCoordinates(0x6e1a, 1, Coordinates(0, 0, 1740)),
    #           #DeviceCoordinates(0x6e2d, 1, Coordinates(10147, -4136, 1780)),
    #           DeviceCoordinates(0x6e32, 1, Coordinates(3385, 5349, 1200)),
    #           DeviceCoordinates(0x6e37, 1, Coordinates(7809, 4967, 1200)),
    #           DeviceCoordinates(0x6e4e, 1, Coordinates(10526, 2008, 1100)),
    #           DeviceCoordinates(0x6e56, 1, Coordinates(-2366, 2743, 1800))]

    anchors = [DeviceCoordinates(0x6e4e, 1, Coordinates(0, 0, 2270)),
               DeviceCoordinates(0x6e32, 1, Coordinates(3021, 7231, 2250)),
               DeviceCoordinates(0x6e1a, 1, Coordinates(0, 2520, 830)),
               DeviceCoordinates(0x6e37, 1, Coordinates(-557, 7813, 1540)),
               DeviceCoordinates(0x6e2d, 1, Coordinates(1400, 0, 2070)),
               DeviceCoordinates(0x6e56, 1, Coordinates(2800, 0, 1800))]

    algorithm = PozyxConstants.POSITIONING_ALGORITHM_UWB_ONLY # positioning algorithm to use
    dimension = PozyxConstants.DIMENSION_3D               # positioning dimension
    height = 1000 # default 1000, height of device, required in 2.5D positioning
    filter_type = PozyxConstants.FILTER_TYPE_MOVING_MEDIAN
    filter_strength = 5

    pozyx = PozyxSerial(serial_port)
    r = ReadyToLocalize(pozyx, osc_udp_client, anchors, algorithm, dimension, height, filter_type, filter_strength, remote_id)
    r.setup()
    while True:
        pub = rospy.Publisher('uwb_pos', Pos, queue_size=1)
        rospy.init_node('talker', anonymous=True)
        while not rospy.is_shutdown():
          r.loop()
