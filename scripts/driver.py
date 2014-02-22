#!/usr/bin/env python
import rospy
from std_msgs.msg import String, Header
from std_srvs.srv import *
from sensor_msgs.msg import NavSatFix, NavSatStatus, Imu
import roscopter.msg
import roscopter.srv
import sys,struct,time,os
import math

##******************************************************************************
# Auto Pilot defines for use throughout the code.  These are custom modes
# defined in teh Arducopter Code under the defines.h file
#*******************************************************************************
STABILIZE = 0                     # hold level position
ACRO = 1                          # rate control
ALT_HOLD = 2                      # AUTO control
AUTO = 3                          # AUTO control
GUIDED = 4                        # AUTO control
LOITER = 5                        # Hold a single location
RTL = 6                           # AUTO control
CIRCLE = 7                        # AUTO control
POSITION = 8                      # AUTO control
LAND = 9                          # AUTO control
OF_LOITER = 10                    # Hold a single location using optical flow

##******************************************************************************
# Parse any arguments that follow the node command
#*******************************************************************************
from optparse import OptionParser
parser = OptionParser("roscopter.py [options]")

parser.add_option("--baudrate", dest="baudrate", type='int',
                  help="master port baud rate", default=115200)
parser.add_option("--device", dest="device", default="/dev/ttyACM0", help="serial device")
parser.add_option("--rate", dest="rate", default=1000, type='int', help="parsing rate for roscopter")
parser.add_option("--mavlink_rate", dest="mavlink_rate", default=10, type='int', help="requested stream rate")
parser.add_option("--source-system", dest='SOURCE_SYSTEM', type='int',
                  default=255, help='MAVLink source system for this GCS')
parser.add_option("--enable-rc-control",dest="enable_rc_control", default=False, help="enable listening to control messages")
parser.add_option("--enable-waypoint-control",dest="enable_waypoint_control", default=True, help="enable listening to waypoint messages")
parser.add_option("--vehicle-name", dest="vehicle_name", default="", help="name of vehicle")
parser.add_option("--default-launch-altitude",dest="default_launch_altitude", default=5, help="default launch altitude in meters")

(opts, args) = parser.parse_args()

##******************************************************************************
# Import and connect to the apm through mavlink
#*******************************************************************************
sys.path.insert(0, os.path.join(os.path.dirname(os.path.realpath(__file__)), '../mavlink/pymavlink'))
import mavutil

# create a mavlink serial instance
master = mavutil.mavlink_connection(opts.device, baud=opts.baudrate)

if opts.device is None:
    print("You must specify a serial device")
    sys.exit(1)


##******************************************************************************
 # Name:    mav_control_cb
 # Function is a placeholder until implemented in the APM
#*******************************************************************************
#def mav_control(data):
#    '''
#    Set roll, pitch and yaw.
#    roll                      : Desired roll angle in radians (float)
#    pitch                     : Desired pitch angle in radians (float)
#    yaw                       : Desired yaw angle in radians (float)
#    thrust                    : Collective thrust, normalized to 0 .. 1 (float)
#    '''    
#    master.mav.set_roll_pitch_yaw_thrust_send(master.target_system, master.target_component,
#                                                                data.roll, data.pitch, data.yaw, data.thrust)
#
#    rospy.loginfo ("sending control: %s"%data)


##******************************************************************************
 # Name:    send_rc_cb
 # Purpose: Callback function for "send_rc" message. Takes incoming 8 integers
 #              and passes to the various PWM RC Channels on the APM. May be
 #              used for direct control of the vehicle. Sending RC values on a 
 #              channel overrides that channel on a physical Transmitter. To
 #              Ignore a channel, a value of -1 or "65535" should be written. To
 #              Return a channel back to a physical transmitter, write a 0. 
 # Params:  data: Desired PWM values for 8 RC Channels
#*******************************************************************************
def send_rc_cb(data):
    master.mav.rc_channels_override_send(master.target_system, master.target_component,data.channel[0],data.channel[1],data.channel[2],data.channel[3],data.channel[4],data.channel[5],data.channel[6],data.channel[7])
    rospy.loginfo ("sending rc: %s"%data)


##******************************************************************************
 # Name:    command_cb
 # Purpose: Callback function for "command" Service.  Specific commands are used
 #              to control functions such as Launch, Land, Arm, Disarm, etc.
 #              All commands may be found within the "APMCommand" Service file.
 #              New commands should be entered there to keep uniform constants
 #              throughout calling functions
 # Params:  data: Requested command variable
#*******************************************************************************
def command_cb(req):
    if req.command == roscopter.srv._APMCommand.APMCommandRequest.CMD_LAUNCH:
        rospy.loginfo ("Launch Command")
        launch()
        #goto_waypoint()
        return True
    elif req.command == roscopter.srv._APMCommand.APMCommandRequest.CMD_LAND:
        rospy.loginfo ("Land Command")
        land()
        return True
    elif req.command == roscopter.srv._APMCommand.APMCommandRequest.CMD_ARM:
        rospy.loginfo ("ARMING")
        master.arducopter_arm()        # One method commented out for the time
        '''master.mav.command_long_send(master.target_system, mavutil.mavlink.MAV_COMP_ID_SYSTEM_CONTROL,
                                 mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 
                                 0, # confirmation
                                 1, # param1 (1 to indicate arm)
                                 0, # param2 (all other params meaningless)
                                 0, # param3
                                 0, # param4
                                 0, # param5
                                 0, # param6
                                 0) # param7
        '''
        return True
    elif req.command == roscopter.srv._APMCommand.APMCommandRequest.CMD_DISARM:
        rospy.loginfo ("DISARMING")
        master.arducopter_disarm()
        '''master.mav.command_long_send(master.target_system, master.target_component,
                                 mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 
                                 0, # confirmation
                                 0, # param1 (1 to indicate dis arm)
                                 0, # param2 (all other params meaningless)
                                 0, # param3
                                 0, # param4
                                 0, # param5
                                 0, # param6
                                 0) # param7
        '''
        return True

    elif req.command == roscopter.srv._APMCommand.APMCommandRequest.CMD_SET_MANUAL:
        rospy.loginfo ("SET MODE TO MANUAL")
        master.mav.set_mode_send(master.target_system, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, MANUAL)
        return True
        
    elif req.command == roscopter.srv._APMCommand.APMCommandRequest.CMD_SET_STABILIZE:
        rospy.loginfo ("SET MODE TO STABILIZE")
        master.mav.set_mode_send(master.target_system, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, STABILIZE)
        return True

    elif req.command == roscopter.srv._APMCommand.APMCommandRequest.CMD_SET_ALT_HOLD:
        rospy.loginfo ("SET MODE TO ALT HOLD")
#        set_current_waypoint(3)
        master.mav.set_mode_send(master.target_system, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, ALT_HOLD)
        return True

    elif req.command == roscopter.srv._APMCommand.APMCommandRequest.CMD_SET_AUTO:
        rospy.loginfo ("SET MODE TO AUTO")
        master.mav.set_mode_send(master.target_system, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, AUTO)
        rospy.sleep(0.1)
        master.mav.set_mode_send(master.target_system, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, AUTO)
        return True

    elif req.command == roscopter.srv._APMCommand.APMCommandRequest.CMD_SET_LOITER:
        rospy.loginfo ("SET MODE TO LOITER")
        master.set_mode_loiter()
        #master.mav.command_long_send(master.target_system, master.target_component,
        #                                 mavutil.mavlink.MAV_CMD_NAV_LOITER_UNLIM, 0, 0, 0, 0, 0, 0, 0, 0)
        #master.mav.set_mode_send(master.target_system, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, LOITER)
        #rospy.sleep(0.1)
        #master.mav.set_mode_send(master.target_system, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, LOITER)
        return 1

    elif req.command == roscopter.srv._APMCommand.APMCommandRequest.CMD_SET_LAND:
        rospy.loginfo ("SET MODE TO LAND")
        master.mav.set_mode_send(master.target_system, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, LAND)
        rospy.sleep(0.1)
        master.mav.set_mode_send(master.target_system, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, LAND)
        return True

    elif req.command == roscopter.srv._APMCommand.APMCommandRequest.RETURN_RC_CONTROL:
        rospy.loginfo ("RETURN RC CONTROL")
        master.mav.rc_channels_override_send(master.target_system, master.target_component,0,0,0,0,0,0,0,0)
        return True

##******************************************************************************
 # Name:    transmit_waypoint
 # Purpose: Function to transmit waypoint. Initally, the function holds until 
 #              mission_request_buffer holds the desired index of waypoint to be
 #              transmitted. This number is designated by the Mavlink 
 #              "MISSION_REQUEST" message.  The waypoint is then transmitted and
 #              its value popped off the buffer.
 # Params:  data: Data to be used to transmit for waypoint
 #              (NOTE): Uses the roscopter.msg.waypoint structure for data
 #          index: The Waypoint Number to be transmitted
 #          wp_type: Type of waypoint item designated by Mavlink:
 #              https://pixhawk.ethz.ch/mavlink/
#*******************************************************************************
def transmit_waypoint(data, index, wp_type):
    while (index not in mission_request_buffer):
        pass

    # Dummy Waypoint
    master.mav.mission_item_send(master.target_system, master.target_component, 
                                 index,              # Waypoint Number
                                 0, 
                                 wp_type,            # Mission Item Type
                                 0,                  # Is Current Waypoint
                                 1,                  # Should Autocontinue to next wp
                                 data.holdTime/1000,      # NAV Command: Hold Time (convert from ms to seconds)
                                 data.posAcc/1000,   # NAV Command: Radius for accept within range (convert from mm to meters)
                                 0,                  # LOITER Command: Orbit to circle (meters). Positive clockwise, Negative counter-clockwise
                                 data.yawFrom/1000,  # NAV/LOITER Command: Yaw Orientation [o to 360 degrees]
                                 data.latitude/1E7,  # local: x position, global: latitude
                                 data.longitude/1E7, # local: y position, global: longitude
                                 data.altitude/1000)  # local: z position, global: altitude (convert from mm to meters)
    mission_request_buffer.pop(0)

    rospy.loginfo ("Waypoint %d: Lat=%f, Lon=%f, Alt=%f, posAcc=%f, holdTime=%f, yawFrom=%f"
        %(index, data.latitude, data.longitude, data.altitude,
          data.posAcc, data.holdTime, data.yawFrom))

##******************************************************************************
 # Name:    waypoint_cb
 # Purpose: Callback function for "waypoint".  Initially transmit dummy point
 #              and then take single waypoint and transmit.  Method follows:
 #              http://qgroundcontrol.org/mavlink/waypoint_protocol
 # Params:  data: Single waypoint to be transmitted
#*******************************************************************************
def waypoint_cb(data):
    rospy.loginfo ("Sending Single Waypoint")

    # Send desired wp radius according to waypoint, divide by 10 to put in cm
    master.mav.param_set_send(master.target_system, master.target_component, "WPNAV_RADIUS",
        data.posAcc/10, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)    

    # Number of waypoints (Plus Dummy Waypoint)
    master.mav.mission_count_send(master.target_system, master.target_component, 2)

    # Send Dummy Waypoint
    dummy_wp = roscopter.msg.Waypoint()
    dummy_wp.latitude = gps_msg.latitude
    dummy_wp.longitude = gps_msg.longitude
    
    transmit_waypoint(dummy_wp, 0, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT)
        
    # Send waypoint
    transmit_waypoint(data, 1, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT)

    # Set current waypoint to be waypoint 1
    set_current_waypoint(1)
    rospy.loginfo ("Waypoint Sent")

##******************************************************************************
 # Name:    waypoint_list_cb
 # Purpose: Callback function for "waypoint_list".  Initially transmit dummy
 #              point and then take list of waypoints from waypoint list and
 #              transmit.  Method follows: 
 #              http://qgroundcontrol.org/mavlink/waypoint_protocol
 # Params:  data: List of Waypoints to be transmitted
#*******************************************************************************
def waypoint_list_cb(data):
    rospy.loginfo ("Sending Waypoint list")

    # Send desired wp radius according to first waypoint of list, divide by 10 to put in cm
    master.mav.param_set_send(master.target_system, master.target_component, "WPNAV_RADIUS",
        data.waypoints[0].posAcc/10, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)

    # If it is desired to store
#    rospy.sleep(1)
#    master.mav.command_long_send(master.target_system, master.target_component,
#                                 mavutil.mavlink.MAV_CMD_PREFLIGHT_STORAGE, 0, 0, 0, 0, 0, 0, 0, 0)

    # Number of waypoints (Plus Dummy Waypoint)
    sizeOfWaypointList = len(data.waypoints)
    master.mav.mission_count_send(master.target_system, master.target_component, sizeOfWaypointList + 1)

    # Send Dummy Waypoint
    dummy_wp = roscopter.msg.Waypoint()
    dummy_wp.latitude = gps_msg.latitude
    dummy_wp.longitude = gps_msg.longitude
    
    transmit_waypoint(dummy_wp, 0, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT)
        
    # Send entire list of waypoints
    rospy.loginfo("*************************************************************")
    for i in range(0, sizeOfWaypointList):
        transmit_waypoint(data.waypoints[i], i+1, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT)

    rospy.loginfo("*************************************************************")

    # Set Current Waypoint
    set_current_waypoint(1)

    rospy.loginfo("Waypoints Sent")


##******************************************************************************
 # Name:    receive_waypoint
 # Purpose: Request information on specific waypoint designated 
 # Params:  num: Value of desired waypoint to be received
#*******************************************************************************
def receive_waypoint(num):
    master.mav.mission_request_send(master.target_system, master.target_component, num)
    rospy.loginfo ("Receive waypoint")

    
##******************************************************************************
 # Name:    clear_waypoints
 # Purpose: Clear all stored waypoints from the APM
#*******************************************************************************
def clear_waypoints():
    master.mav.mission_clear_all_send(master.target_system, master.target_component)
    rospy.loginfo ("Clear Waypoints")


##******************************************************************************
 # Name:    set_current_waypoint
 # Purpose: Set current waypoint of APM according to params
 # Params:  num: Value of Current Waypoint
#*******************************************************************************
def set_current_waypoint(num):
    master.mav.mission_set_current_send(master.target_system, master.target_component, num)
    rospy.loginfo ("Set current waypoint to " + str(num))


##******************************************************************************
 # Name:    num_of_waypoints
 # Purpose: Request the number of waypoints held on APM.  Will be received
 #              through Mavlink "MISSION_COUNT" message
#*******************************************************************************
def num_of_waypoints():
    val = master.mav.mission_request_list_send(master.target_system, master.target_component)

    
# START MISSION
def goto_waypoint():
    master.mav.command_long_send(master.target_system, master.target_component,
                                 mavutil.mavlink.MAV_CMD_MISSION_START, 0, 0,
                                 0, 0, 0,
                                 0, 0, 0)
    
##******************************************************************************
 # Name:    launch
 # Purpose: Send Launch Waypoint.  To trigger Launch, vehicle is set into Auto
 #              mode and then the throttle RC value is slightly increased. 
 #              Immediately after the throttle change, it is given back to the
 #              APM Control.
#*******************************************************************************
def launch():
    rospy.loginfo("Launch Vehicle")
    
    # Number of waypoints (Plus Dummy Waypoint)
    master.mav.mission_count_send(master.target_system, master.target_component, 2)

    # Send Dummy Waypoint
    dummy_wp = roscopter.msg.Waypoint()
    dummy_wp.latitude = gps_msg.latitude
    dummy_wp.longitude = gps_msg.longitude
    
    transmit_waypoint(dummy_wp, 0, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF)
    
    # Send Dummy Waypoint
    dummy_wp = roscopter.msg.Waypoint()
    dummy_wp.latitude = gps_msg.latitude
    dummy_wp.longitude = gps_msg.longitude
    

    launch_wp = roscopter.msg.Waypoint()
    launch_wp.latitude = gps_msg.latitude
    launch_wp.longitude = gps_msg.longitude
    launch_wp.altitude = opts.default_launch_altitude
    transmit_waypoint(launch_wp, 1, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF)

    # Set Launch Waypoint as Current Waypoint
    set_current_waypoint(1)
    
    # Trigger auto mode for launch command
    master.mav.set_mode_send(master.target_system, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, AUTO)
    rospy.sleep(0.1)
    master.mav.set_mode_send(master.target_system, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, AUTO)
    
    rospy.sleep(10)
    # Slightly adjust throttle to trigger auto mode.
    master.mav.rc_channels_override_send(master.target_system, master.target_component, 65535, 65535, 1200, 65535, 65535, 65535, 65535, 65535)
    rospy.sleep(1)
    master.mav.rc_channels_override_send(master.target_system, master.target_component, 0, 0, 0, 0, 0, 0, 0, 0)

    rospy.loginfo ("Vehicle Launched")
    
# Land the vehicle
def land():
    
    master.mav.rc_channels_override_send(master.target_system, master.target_component, 65535, 65535, 1150, 65535, 65535, 65535, 65535, 65535)

    # Land Command
    master.mav.command_long_send(master.target_system, master.target_component,
                                 mavutil.mavlink.MAV_CMD_NAV_LAND, 0, 0,
                                 0, 0, 0,
                                 0, 0, 0)
    rospy.loginfo ("Land Command")


##******************************************************************************
# Publisher and Subscribers to be used for ROS communications of sensor or 
#     Mavlink responses
#*******************************************************************************
# Sensor messages
pub_gps = rospy.Publisher('gps', NavSatFix)
#pub_imu = rospy.Publisher('imu', Imu)
pub_rc = rospy.Publisher('rc', roscopter.msg.RC)
pub_state = rospy.Publisher('state', roscopter.msg.State)
pub_vfr_hud = rospy.Publisher('vfr_hud', roscopter.msg.VFR_HUD)
pub_attitude = rospy.Publisher('attitude', roscopter.msg.Attitude)
pub_raw_imu =  rospy.Publisher('raw_imu', roscopter.msg.Mavlink_RAW_IMU)
pub_status = rospy.Publisher('status', roscopter.msg.Status)
pub_filtered_pos = rospy.Publisher('filtered_pos', roscopter.msg.FilteredPosition)
pub_control_output = rospy.Publisher('controller_output', roscopter.msg.ControllerOutput)
pub_current_mission = rospy.Publisher('current_mission', roscopter.msg.CurrentMission)
pub_mission_item = rospy.Publisher('mission_item', roscopter.msg.MissionItem)

# Allow For RC Control
if opts.enable_rc_control:
    rospy.Subscriber("send_rc", roscopter.msg.RC , send_rc_cb)

# Allow for Waypoint Control
if opts.enable_waypoint_control:
    rospy.Subscriber("waypoint", roscopter.msg.Waypoint , waypoint_cb)
    rospy.Subscriber("waypoint_list", roscopter.msg.WaypointList, waypoint_list_cb)

##******************************************************************************
# Services for APM Commands
#*******************************************************************************
# Allow for commands such as Arm, Disarm, Launch, Land, etc.
rospy.Service("command", roscopter.srv.APMCommand, command_cb)


##******************************************************************************
# Global Message containers
#*******************************************************************************
gps_msg = NavSatFix()
current_mission_msg = roscopter.msg.CurrentMission()

##******************************************************************************
 # Name:    mainloop
 # Purpose: Main loop to initialize ROS node and parse data read from the
 #              Mavlink master.
 # Globals: Publishers and Subscribers
 #          mission_request_buffer: buffer to hold list of requests for desired
 #              mission_item to be sent
#*******************************************************************************
mission_request_buffer = []
def mainloop():
    global gps_msg
    rospy.init_node('roscopter')

    # SEND IF YOU DESIRE A LIST OF ALL PARAMS (TODO: Publish params to a topic)
    #master.mav.param_request_list_send(master.target_system, master.target_component)

    r = rospy.Rate(opts.rate)
    while not rospy.is_shutdown():
        r.sleep()
        msg = master.recv_match(blocking=False)
        if not msg:
            continue

        # Parse incoming message
        if msg.get_type() == "BAD_DATA":
            if mavutil.all_printable(msg.data):
                sys.stdout.write(msg.data)
                sys.stdout.flush()
        else: 
            msg_type = msg.get_type()
            if msg_type == "RC_CHANNELS_RAW" :
                pub_rc.publish([msg.chan1_raw, msg.chan2_raw, msg.chan3_raw, msg.chan4_raw, msg.chan5_raw, msg.chan6_raw, msg.chan7_raw, msg.chan8_raw]) 
            elif msg_type == "HEARTBEAT":
                pub_state.publish(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED, 
                                  msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_GUIDED_ENABLED, 
                                  mavutil.mode_string_v10(msg))
            elif msg_type == "VFR_HUD":
                pub_vfr_hud.publish(msg.airspeed, msg.groundspeed, msg.heading, msg.throttle, msg.alt, msg.climb)

            elif msg_type == "GPS_RAW_INT":
                fix = NavSatStatus.STATUS_NO_FIX
                if msg.fix_type >=3:
                    fix=NavSatStatus.STATUS_FIX

                header = Header()
                header.frame_id = 'base_link'# '/gps'
                header.stamp = rospy.Time.now()

                #rospy.loginfo("Hdop is %d", msg.eph)
                #rospy.loginfo("Vdop is %d", msg.epv)

                sigma = math.sqrt((3.04 * msg.eph**2)**2 + 3.57**2)
                position_covariance = [0] * 9
                position_covariance[0] = sigma #9999
                position_covariance[4] = sigma #9999
                position_covariance[8] = sigma #9999

                pub_gps.publish(NavSatFix(header = header,
                                          latitude = msg.lat/1e07,
                                          longitude = msg.lon/1e07,
                                          altitude = msg.alt/1e03,
                                          position_covariance=position_covariance,
                                          position_covariance_type=NavSatFix.COVARIANCE_TYPE_APPROXIMATED,
                                          status = NavSatStatus(status=fix, service = NavSatStatus.SERVICE_GPS) 
                                          ))

            elif msg_type == "ATTITUDE" :
                pub_attitude.publish(msg.roll, msg.pitch, msg.yaw, msg.rollspeed, msg.pitchspeed, msg.yawspeed)

            elif msg_type == "LOCAL_POSITION_NED" :
                rospy.loginfo("Local Pos: (%f %f %f) , (%f %f %f)" %(msg.x, msg.y, msg.z, msg.vx, msg.vy, msg.vz))

            elif msg_type == "RAW_IMU" :
                pub_raw_imu.publish (Header(), msg.time_usec, 
                                     msg.xacc, msg.yacc, msg.zacc, 
                                     msg.xgyro, msg.ygyro, msg.zgyro,
                                     msg.xmag, msg.ymag, msg.zmag)

            elif msg_type == "SYS_STATUS":
                status_msg = roscopter.msg.Status()
                status_msg.header.stamp = rospy.Time.now()
                status_msg.battery_voltage = msg.voltage_battery
                status_msg.battery_current = msg.current_battery
                status_msg.battery_remaining = msg.battery_remaining
                status_msg.sensors_enabled = msg.onboard_control_sensors_enabled
                pub_status.publish(status_msg)
                
            elif msg_type == "GLOBAL_POSITION_INT":
                header = Header()
                header.stamp = rospy.Time.now()
                pub_filtered_pos.publish(header, msg.lat, msg.lon, msg.alt, 
                                         msg.relative_alt, msg.vx, msg.vy,
                                         msg.vz, msg.hdg)
                                         
            elif msg_type == "NAV_CONTROLLER_OUTPUT":
                current_mission_msg.header.stamp = rospy.Time.now()
                current_mission_msg.wp_dist = msg.wp_dist
                current_mission_msg.target_bearing = msg.target_bearing
                
                pub_current_mission.publish(current_mission_msg)
                pub_control_output.publish(msg.nav_roll, msg.nav_pitch,
                                           msg.nav_bearing, msg.alt_error,
                                           msg.aspd_error, msg.xtrack_error)

            elif msg_type == "MISSION_CURRENT":
                current_mission_msg.header.stamp = rospy.Time.now()
                current_mission_msg.mission_num = msg.seq
                
                pub_current_mission.publish(current_mission_msg)
                
            elif msg_type == "MISSION_ITEM":
                header = Header()
                header.stamp = rospy.Time.now()
                
                pub_mission_item.publish(header, msg.seq, msg.current,
                                         msg.autocontinue, msg.param1,
                                         msg.param2, msg.param3, msg.param4,
                                         msg.x, msg.y, msg.z)

            elif msg_type == "MISSION_COUNT":
                rospy.loginfo ("MISSION_COUNT: Number of Mission Items - " + str(msg.count))
            
            
            elif msg_type == "MISSION_ACK":
                rospy.loginfo ("MISSION_ACK: Mission Message ACK with response - " + str(msg.type))
                
            
            elif msg_type == "COMMAND_ACK":
                rospy.loginfo ("COMMAND_ACK: Command Message ACK with result - " + str(msg.result))
                
                
            elif msg_type == "MISSION_REQUEST":
                rospy.loginfo ("MISSION_REQUEST: Mission Request for target system %d for target component %d with result %d"
                         %(msg.target_system, msg.target_component, msg.seq))
                mission_request_buffer.append(msg.seq)

            elif msg_type == "STATUSTEXT":
                rospy.loginfo ("STATUSTEXT: Status severity is %d. Text Message is %s" %(msg.severity, msg.text)) 

            elif msg_type == "PARAM_VALUE":
                rospy.loginfo ("PARAM_VALUE: ID = %s, Value = %d, Type = %d, Count = %d, Index = %d"
                    %(msg.param_id, msg.param_value, msg.param_type, msg.param_count, msg.param_index))

            #else:
            #    # Message not being processed received
            #    rospy.loginfo( msg_type)




##******************************************************************************
 # Name:    wait_heartbeat
 # Purpose: Function to wait for the heartbeat from the APM
 # Params:  m: mavlink master
#*******************************************************************************
def wait_heartbeat(m):
    '''wait for a heartbeat so we know the target system IDs'''
    print("Waiting for APM heartbeat")
    m.wait_heartbeat()
    print("Heartbeat from APM (system %u component %u)" % (m.target_system, m.target_system))

##******************************************************************************
# MAIN START USING THESE FUNCTIONS
#*******************************************************************************
# wait for the heartbeat msg to find the system ID
wait_heartbeat(master)

# waiting for 10 seconds for the system to be ready
print("Sleeping for 10 seconds to allow system, to be ready")
rospy.sleep(10)
print("Sending all stream request for mavlink_rate %u" % opts.mavlink_rate)

master.mav.request_data_stream_send(master.target_system, master.target_component,
                                    mavutil.mavlink.MAV_DATA_STREAM_ALL, opts.mavlink_rate, 1)
if __name__ == '__main__':
    try:
        # initially clear waypoints and start mainloop
        clear_waypoints()
        mainloop()
    except rospy.ROSInterruptException: pass
