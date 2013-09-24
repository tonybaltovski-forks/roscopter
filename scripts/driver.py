#!/usr/bin/env python
#import roslib; roslib.load_manifest('roscopter')
import rospy
from std_msgs.msg import String, Header
from std_srvs.srv import *
from sensor_msgs.msg import NavSatFix, NavSatStatus, Imu
import roscopter.msg
import roscopter.srv
import sys,struct,time,os

# Auto Pilot modes
# Custom modes defined in ArduCopter defines.h file
# ----------------
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


sys.path.insert(0, os.path.join(os.path.dirname(os.path.realpath(__file__)), '../mavlink/pymavlink'))

# Parse any messages that follow the node command
from optparse import OptionParser
parser = OptionParser("roscopter.py [options]")

parser.add_option("--baudrate", dest="baudrate", type='int',
                  help="master port baud rate", default=115200)
parser.add_option("--device", dest="device", default="/dev/ttyACM0", help="serial device")
parser.add_option("--rate", dest="rate", default=10, type='int', help="requested stream rate")
parser.add_option("--source-system", dest='SOURCE_SYSTEM', type='int',
                  default=255, help='MAVLink source system for this GCS')
parser.add_option("--enable-rc-control",dest="enable_rc_control", default=False, help="Enable listning to control messages")
parser.add_option("--enable-waypoint-control",dest="enable_waypoint_control", default=True, help="Enable listning to waypoint messages")
parser.add_option("--vehicle-name", dest="vehicle_name", default="", help="Name of Vehicle")

(opts, args) = parser.parse_args()


import mavutil

# create a mavlink serial instance
master = mavutil.mavlink_connection(opts.device, baud=opts.baudrate)

if opts.device is None:
    print("You must specify a serial device")
    sys.exit(1)

def wait_heartbeat(m):
    '''wait for a heartbeat so we know the target system IDs'''
    print("Waiting for APM heartbeat")
    m.wait_heartbeat()
    print("Heartbeat from APM (system %u component %u)" % (m.target_system, m.target_system))


#This does not work yet because APM does not have it implemented
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
#    print ("sending control: %s"%data)


def send_rc(data):
    master.mav.rc_channels_override_send(master.target_system, master.target_component,data.channel[0],data.channel[1],data.channel[2],data.channel[3],data.channel[4],data.channel[5],data.channel[6],data.channel[7])
    print ("sending rc: %s"%data)

# Callback for Command Client
# Specific command values are in the service description for APMCommand
def command_callback(req):
    if req.command == roscopter.srv._APMCommand.APMCommandRequest.CMD_LAUNCH:
        print ("Launch Command")
        launch()
        #goto_waypoint()
        return True
    elif req.command == roscopter.srv._APMCommand.APMCommandRequest.CMD_LAND:
        print ("Land Command")
        land()
        return True
    elif req.command == roscopter.srv._APMCommand.APMCommandRequest.CMD_ARM:
        print ("ARMING")
#        master.arducopter_arm()        # One method commented out for the time
        master.mav.command_long_send(master.target_system, mavutil.mavlink.MAV_COMP_ID_SYSTEM_CONTROL,
                                 mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 
                                 0, # confirmation
                                 1, # param1 (1 to indicate arm)
                                 0, # param2 (all other params meaningless)
                                 0, # param3
                                 0, # param4
                                 0, # param5
                                 0, # param6
                                 0) # param7
        return True
    elif req.command == roscopter.srv._APMCommand.APMCommandRequest.CMD_DISARM:
        print ("DISARMING")
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
        
    elif req.command == roscopter.srv._APMCommand.APMCommandRequest.CMD_SET_STABILIZE:
        print ("SET MODE TO STABILIZE")
        master.mav.set_mode_send(master.target_system, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, STABILIZE)
        return True

    elif req.command == roscopter.srv._APMCommand.APMCommandRequest.CMD_SET_ALT_HOLD:
        print ("SET MODE TO ALT HOLD")
        master.mav.set_mode_send(master.target_system, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, ALT_HOLD)
        return True

    elif req.command == roscopter.srv._APMCommand.APMCommandRequest.CMD_SET_AUTO:
        print ("SET MODE TO AUTO")
        master.mav.set_mode_send(master.target_system, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, AUTO)
        rospy.sleep(0.1)
        master.mav.set_mode_send(master.target_system, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, AUTO)
        return True

    elif req.command == roscopter.srv._APMCommand.APMCommandRequest.CMD_SET_LOITER:
        print ("SET MODE TO LOITER")
        master.set_mode_loiter()
        #master.mav.command_long_send(master.target_system, master.target_component,
        #                                 mavutil.mavlink.MAV_CMD_NAV_LOITER_UNLIM, 0, 0, 0, 0, 0, 0, 0, 0)
        #master.mav.set_mode_send(master.target_system, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, LOITER)
        #rospy.sleep(0.1)
        #master.mav.set_mode_send(master.target_system, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, LOITER)
        return 1

    elif req.command == roscopter.srv._APMCommand.APMCommandRequest.CMD_SET_LAND:
        print ("SET MODE TO LAND")
        master.mav.set_mode_send(master.target_system, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, LAND)
        rospy.sleep(0.1)
        master.mav.set_mode_send(master.target_system, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, LAND)
        return True

    elif req.command == roscopter.srv._APMCommand.APMCommandRequest.RETURN_RC_CONTROL:
        print ("RETURN RC CONTROL")
        master.mav.rc_channels_override_send(master.target_system, master.target_component,0,0,0,0,0,0,0,0)
        return True

# Send a waypoint collected from Waypoint Message
def send_waypoint(data):
    # Number of waypoints (Plus Dummy Waypoint)
    master.mav.mission_count_send(master.target_system, master.target_component, 2)
    
    # Dummy Waypoint
    master.mav.mission_item_send(master.target_system, master.target_component, 
                                 0,     # Waypoint Number
                                 0, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                                 0,     # Is Current Waypoint
                                 0,     # Should Autocontinue to next wp
                                 0,     # NAV Command: Radius for accept within range (meters)
                                 0,     # NAV Command: Hold Time (ms)
                                 0,     # LOITER Command: Orbit to circle (meters).  Positive clockwise, Negative counter-clockwise
                                 0,     # NAV/LOITER Command: Yaw Orientation [o to 360 degrees]
                                 gps_msg.latitude,  # local: x position, global: latitude
                                 gps_msg.longitude, # local: y position, global: longitude
                                 0)          # local: z position, global: altitude
    rospy.sleep(1)

    # Waypoint to be sent
    print ("Waypoint to be sent: Lat=%f, Lon=%f, Alt=%f, posAcc=%f, holdTime=%f, yawFrom=%f"
            %(data.latitude, data.longitude, data.altitude, data.posAcc, data.holdTime,
              data.yawFrom))

    master.mav.mission_item_send(master.target_system, master.target_component, 
                                 1,     # Waypoint Number
                                 0, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                                 1,     # Is Current Waypoint
                                 0,     # Should Autocontinue to next wp
                                 data.posAcc,     # NAV Command: Radius for accept within range (meters)
                                 data.holdTime,     # NAV Command: Hold Time (ms)
                                 0,     # LOITER Command: Orbit to circle (meters).  Positive clockwise, Negative counter-clockwise
                                 data.yawFrom,     # NAV/LOITER Command: Yaw Orientation [o to 360 degrees]
                                 data.latitude,     # local: x position, global: latitude
                                 data.longitude,     # local: y position, global: longitude
                                 data.altitude)    # local: z position, global: altitude

    rospy.sleep(1)

    # Set current waypoint to be waypoint 1
    set_current_waypoint()
    receive_waypoint(1)
    print ("Sending waypoint")

def receive_waypoint(num):
    master.mav.mission_request_send(master.target_system, master.target_component, num)
    print ("Receive waypoint")
    

def clear_waypoints():
    master.mav.mission_clear_all_send(master.target_system, master.target_component)
    print ("Clear Waypoints")

def set_current_waypoint():
    master.mav.mission_set_current_send(master.target_system, master.target_component, 1)
    print ("Set current waypoint")

def num_of_waypoints():
    val = master.mav.mission_request_list_send(master.target_system, master.target_component)
    print ("Num of Waypoints")
    
# START MISSION
def goto_waypoint():
    master.mav.command_long_send(master.target_system, master.target_component,
                                 mavutil.mavlink.MAV_CMD_MISSION_START, 0, 0,
                                 0, 0, 0,
                                 0, 0, 0)
    
# Launch the vehicle
def launch():
    
    master.mav.mission_count_send(master.target_system, master.target_component, 2)

    # Dummy Waypoint
    master.mav.mission_item_send(master.target_system, master.target_component, 
                                 0, 0, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                                 0, 0, 0,
                                 0, 0, 0, 
                                 0, 0, 0)
    
    rospy.sleep(1)

    # Launch Waypoint
    master.mav.mission_item_send(master.target_system, master.target_component, 
                                 1,     # Waypoint Number
                                 0, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                                 1,     # Is Current Waypoint
                                 0,     # Should Autocontinue to next wp
                                 0,     # NAV Command: Radius for accept within range (meters)
                                 0,     # NAV Command: Hold Time (ms)
                                 0,     # LOITER Command: Orbit to circle (meters).  Positive clockwise, Negative counter-clockwise
                                 270,     # NAV/LOITER Command: Yaw Orientation [o to 360 degrees]
                                 0,     # local: x position, global: latitude
                                 0,     # local: y position, global: longitude
                                 10)    # local: z position, global: altitude   (meters)

    rospy.sleep(1)
    # Set Launch Waypoint as Current Waypoint
    set_current_waypoint()
    rospy.sleep(1)
    receive_waypoint(1)
    
    # Trigger auto mode for launch command
    master.mav.set_mode_send(master.target_system, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, AUTO)
    rospy.sleep(0.1)
    master.mav.set_mode_send(master.target_system, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, AUTO)
    
    rospy.sleep(10)
    # Slightly adjust throttle to trigger auto mode.
    master.mav.rc_channels_override_send(master.target_system, master.target_component, 65535, 65535, 1200, 65535, 65535, 65535, 65535, 65535)
    rospy.sleep(1)
    master.mav.rc_channels_override_send(master.target_system, master.target_component, 0, 0, 0, 0, 0, 0, 0, 0)

    print ("Launch Command")
    
# Land the vehicle
def land():
    
    master.mav.rc_channels_override_send(master.target_system, master.target_component, 65535, 65535, 1150, 65535, 65535, 65535, 65535, 65535)

    # Land Command
    master.mav.command_long_send(master.target_system, master.target_component,
                                 mavutil.mavlink.MAV_CMD_NAV_LAND, 0, 0,
                                 0, 0, 0,
                                 0, 0, 0)
    print ("Land Command")

# Test Mission
def send_test_mission():
    # Number of waypoints (Plus Dummy Waypoint)
    master.mav.mission_count_send(master.target_system, master.target_component, 5)
    
    # Dummy Waypoint
    master.mav.mission_item_send(master.target_system, master.target_component, 
                                 0,     # Waypoint Number
                                 0, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                                 0,     # Is Current Waypoint
                                 0,     # Should Autocontinue to next wp
                                 0,     # NAV Command: Radius for accept within range (meters)
                                 0,     # NAV Command: Hold Time (ms)
                                 0,     # LOITER Command: Orbit to circle (meters).  Positive clockwise, Negative counter-clockwise
                                 0,     # NAV/LOITER Command: Yaw Orientation [o to 360 degrees]
                                 29.662180,  # local: x position, global: latitude
                                 -82.377823, # local: y position, global: longitude
                                 0)          # local: z position, global: altitude
    rospy.sleep(1)

    # Waypoint 1
    master.mav.mission_item_send(master.target_system, master.target_component, 
                                 1,     # Waypoint Number
                                 0, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                                 1,     # Is Current Waypoint
                                 0,     # Should Autocontinue to next wp
                                 0,     # NAV Command: Radius for accept within range (meters)
                                 0,     # NAV Command: Hold Time (ms)
                                 0,     # LOITER Command: Orbit to circle (meters).  Positive clockwise, Negative counter-clockwise
                                 0,     # NAV/LOITER Command: Yaw Orientation [o to 360 degrees]
                                 0,     # local: x position, global: latitude
                                 0,     # local: y position, global: longitude
                                 45)    # local: z position, global: altitude

    rospy.sleep(1)

    # Waypoint 2
    master.mav.mission_item_send(master.target_system, master.target_component, 
                                 2, 0, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                                 0, 1, 100,
                                 10, 43, 23, 
                                 20, -72, 25)
    rospy.sleep(1)
    master.mav.mission_item_send(master.target_system, master.target_component, 
                                 3, 0, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                                 0, 0, 1,
                                 2, 3, 4, 
                                 29.100, -82.005, 50)
    rospy.sleep(1)
    master.mav.mission_item_send(master.target_system, master.target_component, 
                                 4, 0, mavutil.mavlink.MAV_CMD_NAV_LAND,
                                 0, 0, 0,
                                 0, 0, 0,
                                 0, 0, 0)
    rospy.sleep(1)
    set_current_waypoint()

    num_of_waypoints()
    receive_waypoint(0)
    receive_waypoint(1)
    receive_waypoint(2)
    receive_waypoint(3)
    receive_waypoint(4)
    print ("Send Test Mission")


pub_gps = rospy.Publisher('gps', NavSatFix)
#pub_imu = rospy.Publisher('imu', Imu)
pub_rc = rospy.Publisher('rc', roscopter.msg.RC)
pub_state = rospy.Publisher('state', roscopter.msg.State)
pub_vfr_hud = rospy.Publisher('vfr_hud', roscopter.msg.VFR_HUD)
pub_attitude = rospy.Publisher('attitude', roscopter.msg.Attitude)
pub_raw_imu =  rospy.Publisher('raw_imu', roscopter.msg.Mavlink_RAW_IMU)
pub_status = rospy.Publisher('/apm/status', roscopter.msg.Status)
pub_filtered_pos = rospy.Publisher('/apm/filtered_pos', roscopter.msg.FilteredPosition)
pub_control_output = rospy.Publisher('/apm/controller_output', roscopter.msg.ControllerOutput)
pub_current_mission = rospy.Publisher('/apm/current_mission', roscopter.msg.CurrentMission)
pub_mission_item = rospy.Publisher('/apm/mission_item', roscopter.msg.MissionItem)

if opts.enable_rc_control:
    rospy.Subscriber("/apm/control", roscopter.msg.Control , mav_control)
    rospy.Subscriber("/apm/send_rc", roscopter.msg.RC , send_rc)

if opts.enable_waypoint_control:
    rospy.Service("/apm/command", roscopter.srv.APMCommand, command_callback)
    rospy.Subscriber("/apm/waypoint", roscopter.msg.Waypoint , send_waypoint)

#state
gps_msg = NavSatFix()
current_mission_msg = roscopter.msg.CurrentMission()


def mainloop():
    rospy.init_node('roscopter')
    while not rospy.is_shutdown():
        rospy.sleep(0.001)
        msg = master.recv_match(blocking=False)
        if not msg:
            continue
        #print msg.get_type()
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
                pub_gps.publish(NavSatFix(latitude = msg.lat/1e07,
                                          longitude = msg.lon/1e07,
                                          altitude = msg.alt/1e03,
                                          status = NavSatStatus(status=fix, service = NavSatStatus.SERVICE_GPS) 
                                          ))
            #pub.publish(String("MSG: %s"%msg))
            elif msg_type == "ATTITUDE" :
                pub_attitude.publish(msg.roll, msg.pitch, msg.yaw, msg.rollspeed, msg.pitchspeed, msg.yawspeed)


            elif msg_type == "LOCAL_POSITION_NED" :
                print "Local Pos: (%f %f %f) , (%f %f %f)" %(msg.x, msg.y, msg.z, msg.vx, msg.vy, msg.vz)

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
                                           
                #clear_waypoints()
                #num_of_waypoints()

            elif msg_type == "MISSION_CURRENT":
                current_mission_msg.header.stamp = rospy.Time.now()
                current_mission_msg.mission_num = msg.seq
                
                pub_current_mission.publish(current_mission_msg)
                
            elif msg_type == "MISSION_ITEM":
                header = Header()
                header.stamp = rospy.Time.now()
                
                pub_mission_item.publish(header, msg.seq, msg.current,
                                         msg.autocontinue, msg.param2,
                                         msg.param1, msg.param3, msg.param4,
                                         msg.x, msg.y, msg.z)


            elif msg_type == "MISSION_COUNT":
                print ("MISSION_COUNT: Number of Mission Items - " + str(msg.count))
            
            
            elif msg_type == "MISSION_ACK":
                print ("MISSION_ACK: Mission Message ACK with response - " + str(msg.type))
                
            
            elif msg_type == "COMMAND_ACK":
                print ("COMMAND_ACK: Command Message ACK with result - " + str(msg.result))
                
                
            elif msg_type == "MISSION_REQUEST":
                print ("MISSION_REQUEST: Mission Request for target system %d for target component %d with result %d"
                         %(msg.target_system, msg.target_component, msg.seq))
                
                
            elif msg_type == "STATUSTEXT":
                print ("STATUSTEXT: Status severity is %d. Text Message is %s" %(msg.severity, msg.text)) 
                

            #else:
            #    # Message not being processed received
            #    print msg_type


# wait for the heartbeat msg to find the system ID
wait_heartbeat(master)


# waiting for 10 seconds for the system to be ready
print("Sleeping for 10 seconds to allow system, to be ready")
rospy.sleep(10)
print("Sending all stream request for rate %u" % opts.rate)
#for i in range(0, 3):

master.mav.request_data_stream_send(master.target_system, master.target_component,
                                    mavutil.mavlink.MAV_DATA_STREAM_ALL, opts.rate, 1)
master.arducopter_arm()


#master.mav.set_mode_send(master.target_system, 
if __name__ == '__main__':
    try:
        clear_waypoints()
        mainloop()
    except rospy.ROSInterruptException: pass
