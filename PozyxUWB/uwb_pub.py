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
from time import sleep
import time
from geometry_msgs.msg import PointStamped,Point
from dji_sdk_demo.msg import Pos
from pypozyx import *
from pythonosc.osc_message_builder import OscMessageBuilder
from pythonosc.udp_client import SimpleUDPClient
"""
posdata=[[0,0,0],[0,0,0],[0,0,0]]
realdata=[[0,0,0],[0,0,0],[0,0,0]]
startPos=[0,0,0]

a=[1,-1.279632424997809,0.477592250072517]
b=[0.049489956268677,0.098979912537354,0.049489956268677]
#a=[1,-1.705552145544084,0.743655195048866]
#b=[0.009525762376195,0.019051524752391,0.009525762376195]
"""
N=1000
i=0
DT=0
class ReadyToLocalize(object):
    """Continuously calls the Pozyx positioning function and prints its position."""

    def pos_pub(self):
        pub = rospy.Publisher('uwb_pos', Pos, queue_size=1)
        rospy.init_node('talker', anonymous=True)

        while not rospy.is_shutdown():
            n1=rospy.Time.now().nsecs
            position = Coordinates()
            status = self.pozyx.doPositioning(position, self.dimension, self.height, self.algorithm, remote_id=self.remote_id)
            curr_pos=Pos()
            curr_pos.x=position.x/1000.0
            curr_pos.y=position.y/1000.0
            curr_pos.z=position.z/1000.0
            #curr_pos.time=rospy.Time.now().nsecs-n1
            pub.publish(curr_pos)
         

    def __init__(self, pozyx, osc_udp_client, anchors, algorithm=POZYX_POS_ALG_UWB_ONLY, dimension=POZYX_3D, height=1000, remote_id=None):
        self.pozyx = pozyx
        self.osc_udp_client = osc_udp_client
       
        self.anchors = anchors
        self.algorithm = algorithm
        self.dimension = dimension
        self.height = height
        self.remote_id = remote_id

        
    def setup(self):
        pos_error=0
        """Sets up the Pozyx for positioning by calibrating its anchor list."""
        print("------------POZYX POSITIONING V1.1 -------------")
        print("NOTES: ")
        print("- No parameters required.")
        print()
        print("- System will auto start configuration")
        print()
        print("- System will auto start positioning")
        print()
        self.pozyx.printDeviceInfo(self.remote_id)
        print()
        print("------------POZYX POSITIONING V1.1 --------------")
        print()
        self.pozyx.clearDevices(self.remote_id)

        self.setAnchorsManual()
        self.printPublishConfigurationResult()

        
        self.pozyx.setPositionFilter(FILTER_TYPE_MOVINGAVERAGE,0xA)
       # self.pozyx.setRangingProtocol(0x0)
    

    def setAnchorsManual(self):
        """Adds the manually measured anchors to the Pozyx's device list one for one."""
        status = self.pozyx.clearDevices(self.remote_id)
        for anchor in self.anchors:
            status &= self.pozyx.addDevice(anchor, self.remote_id)
        if len(self.anchors) > 4:
            status &= self.pozyx.setSelectionOfAnchors(POZYX_ANCHOR_SEL_AUTO, len(self.anchors))
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


if __name__ == "__main__":
    # shortcut to not have to find out the port yourself
    serial_port = get_first_pozyx_serial_port()
    if serial_port is None:
        print("No Pozyx connected. Check your USB cable or your driver!")
        quit()

    remote_id = 0x6069                 # remote device network ID
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

    
    anchors = [DeviceCoordinates(0x6e37, 1, Coordinates(0, 0 ,1500)),
               DeviceCoordinates(0x6e2d, 1, Coordinates(0 , 4870, 1800)),
               DeviceCoordinates(0x6e32, 1, Coordinates(10100, 4880,1200)),
               DeviceCoordinates(0x6e56, 1, Coordinates(10000, 0, 1800))]


#    anchors = [DeviceCoordinates(0x6e4e, 1, Coordinates(10940 , 0 ,2220)),
#               DeviceCoordinates(0x6e2d, 1, Coordinates(2660 , 5250,1125)),
#               DeviceCoordinates(0x6e1a, 1, Coordinates(0, 0,2220)),
#               DeviceCoordinates(0x6e37, 1, Coordinates(7880,5300,1125)),
#               DeviceCoordinates(0x6e56, 1, Coordinates(13430, 2700, 1110)),
#               DeviceCoordinates(0x6e32, 1, Coordinates(5000,0,160))]
#    anchors111 = [DeviceCoordinates(0x6e2d, 1, Coordinates(2500,3880,1760)),
#               DeviceCoordinates(0x6e37, 1, Coordinates(4075 , 0 ,1490)),
#               DeviceCoordinates(0x6e1a, 1, Coordinates(560, 1305, 900)),
#               DeviceCoordinates(0x6e4e, 1, Coordinates(4390, 3880,1500)),
#               DeviceCoordinates(0x6e32, 1, Coordinates(2200,1200,1400))]
   
    algorithm = POZYX_POS_ALG_UWB_ONLY # positioning algorithm to use
    dimension = POZYX_3D               # positioning dimension
    height = 1000                      # height of device, required in 2.5D positioning

    pozyx = PozyxSerial(serial_port)
    r = ReadyToLocalize(pozyx, osc_udp_client, anchors, algorithm, dimension, height, remote_id)
    r.setup()
    i=0
    
    while True:
        try:
            r.pos_pub()
        except rospy.ROSInterruptException:
            pass

