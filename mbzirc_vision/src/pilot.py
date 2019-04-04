""" This module controls the trajectory of the drone on the basis of inputs """

import math
import threading
import time
from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil # Needed for command message definitions
from queue import Queue

EARTH_D = 1.113195e5    # [m] diameter of earth (ARE WE SURE???)

def saturate(value, minimum, maximum, value_zero):
    """ TODO: aggiungere documentazione """

    if value > maximum:
        return maximum
    if value < minimum:
        return minimum
    if value < value_zero and value > -value_zero:
        return 0.0
    return value


def get_distance_meters(location1, location2):
    """ TODO: aggiungere documentazione """

    return EARTH_D * math.hypot(location2.lat - location1.lat, \
                                location2.lon - location1.lon)


class Pilot(threading.Thread):
    """ Controls the motion of the drone """

    # Parameters
    gnd_speed = 2.5    # [m/s] speed of AUTO mode
    vel_x_min = -1.0   # [m/s] minimum speed along x
    vel_x_max = 1.0    # [m/s] maximum speed along x
    vel_y_min = -2.5   # [m/s] minimum speed along y
    vel_y_max = 2.5    # [m/s] maximum speed along y
    vel_zero = 0.1     # [m/s] experimental speed value when copter is considered still
    dist_max = 25      # [m] sligthly greater than max dist for positive target recognition
    dist_min = 8 - vel_zero / vel_x_max * dist_max  # [m] minimum distance to stop
    err_x_max = 4.8 / (2 * 2.8) * dist_max # [m] max err x for positive target recognition
                            # (4.8mm Vision Sensor Width; 2.8mm Vision Sensor Focal Lenght)
    kerr_velx = vel_x_max / dist_max   # [1/sec] control gain error speed along x body
    kerr_vely = vel_y_max / err_x_max  # [1/sec] control gain error speed along y body
    alt_min = 3        # [m] minimum altitude during balloon reaching mission
    alt_max = 15       # [m] maximum altitude during balloon reaching mission
    delta_alt = 5      # [m] altitude increase to leave target out of camera scope after ACTION
    target_dist = None # [m] distance of the target (when detected)
    target_loc = None  # [m] position of the target (when detected)
    mode = 'GROUND'
    RC_PWM_LIMIT = 1800 # 1800 on SITL, 1400 on BioDrone (SC Radio 5: 982 stabilize; 1494 alt_hold; 2006 pos_hold)


    def __init__(self, name):
        threading.Thread.__init__(self)
        self.__stop_event = threading.Event()
        self.name = name
        self.imgbuffer = Queue(1)
        self.vehicle = None


    def stop(self):
        """ Stop the pilot thread """
        self.__stop_event.set()


    def stopped(self):
        """ Return true if the pilot thread should stop """
        return self.__stop_event.is_set()


    def close(self):
        """ Start closing the pilot """

        self.stop()
        self.join()


    def connect_to_vehicle(self):
        """ Connect to the vehicle """

        print "Connecting to vehicle..."
        self.vehicle = connect('127.0.0.1:14550', wait_ready=True)


    def close_vehicle(self):
        """ Close the vehicle """

        self.vehicle.close()


    def calibrate_altitude_via_pressure(self):
        """ Procedure to calibrate altitude and altitude ref through
        arming-disarming/arming-disarming/arming-disarming
        and pressure sensor calibration
        """

        self.vehicle.mode = VehicleMode("STABILIZE")
        print "Arming-Disarming/Arming-Disarming/Arming-Disarming"
        self.vehicle.armed = True
        time.sleep(5)
        self.vehicle.armed = False
        time.sleep(5)
        self.vehicle.armed = True
        time.sleep(5)
        self.vehicle.armed = False
        time.sleep(5)
        self.vehicle.armed = True
        time.sleep(5)
        self.vehicle.armed = False
        time.sleep(5)

        print "Pressure sensor calibration..."

        msg = self.vehicle.message_factory.command_long_encode(
            0,      # target_system
            0,      # target_component
            mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION, #command
            0,      # confirmation
            0,
            0,
            1,      # param 3, ground pressure calibration
            0,
            0,
            0,
            0)

        # Send command to vehicle
        self.vehicle.send_mavlink(msg)
        self.vehicle.flush()

        time.sleep(10)
        print "Pressure sensor calibration done!"


    def arm_and_takeoff(self, target_altitude):
        """ Arms vehicle and fly to target_altitude. """

        print "Basic pre-arm checks"
        # Don't try to arm until autopilot is ready
        while not self.vehicle.is_armable:
            print " Waiting for vehicle to initialize..."
            time.sleep(1)

        self.calibrate_altitude_via_pressure()

        self.vehicle.mode = VehicleMode("GUIDED")
        self.vehicle.armed = True

        # Confirm vehicle armed before attempting to take off
        while not self.vehicle.armed:
            print " Waiting for arming..."
            time.sleep(1)

        print "Taking off!"
        # Take off to target altitude
        self.vehicle.simple_takeoff(target_altitude)

        # Wait until the vehicle reaches a safe height before processing the mission,
        # otherwise the command after Vehicle.simple_takeoff will execute immediately.
        while True:
            print " Altitude: ", self.vehicle.location.global_relative_frame.alt
            # Break and return from function just below target altitude
            if self.vehicle.location.global_relative_frame.alt >= target_altitude * 0.95:
                print "Reached target altitude"
                break
            time.sleep(1)


    def condition_yaw(self, heading, relative=False, direction=True):
        """ TODO: aggiungere documentazione """

        # Create the CONDITION_YAW command using command_long_encode()
        msg = self.vehicle.message_factory.command_long_encode(
            0,      # target system
            0,      # target component
            mavutil.mavlink.MAV_CMD_CONDITION_YAW, #command
            0,      #confirmation
            heading,    # param 1, yaw in degrees
            0,          # param 2, yaw speed deg/s
            1 if direction else -1, # param 3, yaw direction ccw -1, cw 1
            1 if relative else 0,   # param 4, yaw relative to travel direction 1, absolute angle 0
            0,      # param 5 ~ 7 not used
            0,
            0)
        # Send command to vehicle
        self.vehicle.send_mavlink(msg)
        self.vehicle.flush()



    def leveling_and_orienting(self, target_dx, target_dy, target_dist_normal, \
                               target_yaw, direction=True):
        """ TODO: aggiungere documentazione """

        my_latit = self.vehicle.location.global_relative_frame.lat
        my_longit = self.vehicle.location.global_relative_frame.lon
        my_altit = self.vehicle.location.global_relative_frame.alt
        my_psi = self.vehicle.heading
        target_dist = math.hypot(target_dist_normal, target_dx)
        bearing = (my_psi + target_yaw) % 360.0
        d_latit = target_dist * math.cos((bearing) * math.pi / 180) / EARTH_D
        d_longit = target_dist * math.sin((bearing) * math.pi / 180) / EARTH_D
        target_latit = my_latit  + d_latit
        target_longit = my_longit + d_longit
        target_altit = saturate(my_altit - target_dy, self.alt_min, self.alt_max, 0)
        target_loc = LocationGlobalRelative(target_latit, target_longit, target_altit)
        my_loc_leveled = LocationGlobalRelative(my_latit, my_longit, target_altit)

        print "Orienting yaw..."
        self.condition_yaw(bearing, direction=direction)
        print "Yaw oriented!"
        time.sleep(5)

        self.vehicle.simple_goto(my_loc_leveled)

        while True:
            print "Leveling target altitude: ", self.vehicle.location.global_relative_frame.alt
            #Break and return from function just below and upper target altitude
            if self.vehicle.location.global_relative_frame.alt >= my_loc_leveled.alt * 0.95 and \
               self.vehicle.location.global_relative_frame.alt <= my_loc_leveled.alt * 1.05:
                print "Target altitude leveled!"
                break
            time.sleep(1)

        return target_loc, target_dist


    def send_ned_velocity(self, v_x, v_y, v_z):
        """ TODO: aggiungere documentazione """

        msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
            0,      # time_boot_ms (not used)
            0,      # target system
            0,      # target component
            mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
            0b0000111111000111,     # type_mask (only speeds enabled)
            0,      # x (not used)
            0,      # y (not used)
            0,      # z (not used)
            v_x,    # x velocity (m/s)
            v_y,    # y velocity
            v_z,    # z velocity
            0,      # x acceleration (not supported yet, ignored in GCS_Mavlink)
            0,      # y acceleration
            0,      # z acceleration
            0,      # yaw (not supported yet, ignored in GCS_Mavlink)
            0)      # yaw_rate (not supported yet, ignored in GCS_Mavlink)

        self.vehicle.send_mavlink(msg)
        self.vehicle.flush()



    def set_velocity_body(self, v_x, v_y, v_z):
        """ TODO: aggiungere documentazione """

        msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
            0,      # time_boot_ms (not used)
            0,      # target system
            0,      # target component
            mavutil.mavlink.MAV_FRAME_BODY_NED, # frame
            0b0000111111000111,     # type_mask (only speeds enabled)
            0,      # x (not used)
            0,      # y (not used)
            0,      # z (not used)
            v_x,    # x velocity (m/s)
            v_y,    # y velocity
            v_z,    # z velocity
            0,      # x acceleration (not supported yet, ignored in GCS_Mavlink)
            0,      # y acceleration
            0,      # z acceleration
            0,      # yaw (not supported yet, ignored in GCS_Mavlink)
            0)      # yaw_rate (not supported yet, ignored in GCS_Mavlink)
        self.vehicle.send_mavlink(msg)
        self.vehicle.flush()


    def clear_mission(self):
        """ TODO: aggiungere documentazione """

        self.vehicle.commands.clear()
        self.vehicle.commands.upload()
        self.download_mission()


    def download_mission(self):
        """ TODO: aggiungere documentazione """

        cmds = self.vehicle.commands
        cmds.download()
        time.sleep(2)
        cmds.wait_ready()


    def get_current_mission(self):
        """ TODO: aggiungere documentazione """

        print "Downloading mission..."
        self.download_mission()
        mission_list = []
        n_wp = 0
        if self.vehicle.commands.count > 0:
            for waypoint in self.vehicle.commands:
                mission_list.append(waypoint)
                n_wp += 1
        return n_wp, mission_list


    def put_input(self, proc_frame):
        """ TODO: aggiungere documentazione """
        if not self.imgbuffer.full():
            self.imgbuffer.put(proc_frame)


    def get_input(self):
        """ TODO: aggiungere documentazione """
        return self.imgbuffer.get()



    def run(self):

        def get_bearing(location_1, location_2):
            """ TODO: aggiungere documentazione """
            dlat = location_2.lat - location_1.lat
            dlon = location_2.lon - location_1.lon
            bearing = 180 / math.pi * math.atan2(dlon, dlat)
            if bearing < 0.0:
                bearing += 360
            return bearing

        self.connect_to_vehicle()
        self.clear_mission()

        while True:
            # Stop control if thread is asked to do so
            if self.stopped():
                break

            # Stop control if the human pilot wants 'manual control'
            if self.vehicle.channels['5'] > self.RC_PWM_LIMIT:
                break

            if self.mode == 'GROUND':

                n_wp, mission_list = self.get_current_mission()

                if n_wp > 0:
                    print "A valid mission has been uploaded!..."
                    #balloon_count = 0  #unused
                    #n_balloons = 1     #unused

                    print "Set new home location to current location"
                    self.vehicle.home_location = self.vehicle.location.global_frame
                    self.download_mission()
                    print "Home location: %s" % self.vehicle.home_location
                    time.sleep(5)

                    self.mode = 'TAKEOFF'
                    print "CC State Machine Mode switching to: TAKEOFF"

            elif self.mode == 'TAKEOFF':

                self.arm_and_takeoff(10)
                self.vehicle.mode = VehicleMode("AUTO")
                self.vehicle.groundspeed = self.gnd_speed
                self.mode = 'MISSION SEARCHING'
                print "CC State Machine Mode switching to: MISSION SEARCHING"

            elif self.mode == 'MISSION SEARCHING':

                print "Current WP: %d of %d " \
                      %(self.vehicle.commands.next, self.vehicle.commands.count)

                (locked, _, _, err_x_m, err_y_m, dist, psi_err, _) = self.get_input()[1:]

                #TODO: controllo se None

                if locked == 1 and dist < self.dist_max:
                    print "Balloon detected at distance %d m." % (dist)
                    self.vehicle.mode = VehicleMode("GUIDED")
                    print "Waiting for LEVELING and ORIENTING..."
                    time.sleep(10)
                    (self.target_loc, self.target_dist) = self.leveling_and_orienting(
                        err_x_m, err_y_m, dist, psi_err, direction=True)
                    print "Waiting before going to REACHING..."
                    time.sleep(10)
                    self.mode = 'MISSION REACHING'
                    print "CC State Machine Mode switching to: MISSION REACHING"

                if self.vehicle.commands.next == 5:
                    print "Final WP reached!"
                    self.vehicle.mode = VehicleMode("RTL")
                    self.mode = 'BACK'
                    print "CC State Machine Mode switching to: BACK to home"

            elif self.mode == 'MISSION REACHING':

                (locked, _, _, err_x_m, err_y_m, dist, psi_err, _) = self.get_input()[1:]
                #TODO: controllo se None

                if dist > self.target_dist:
                    locked = 0

                if locked == 1:
                    vel_x = self.kerr_velx * (dist - self.dist_min)
                    vel_x = saturate(vel_x, self.vel_x_min, self.vel_x_max, self.vel_zero)
                    vel_y = self.kerr_vely * err_x_m
                    vel_y = saturate(vel_y, self.vel_y_min, self.vel_y_max, self.vel_zero)
                    self.set_velocity_body(vel_x, vel_y, 0)
                    print "Vx = %.2f m/sec" % (vel_x), "; Vy = %.2f m/sec" % (vel_y)
                    if vel_x == 0:
                        print "Balloon reached!"
                        print "CC State Machine Mode switching to: MISSION ACTION"
                        self.mode = 'MISSION ACTION'
                else:
                    my_latit = self.vehicle.location.global_relative_frame.lat
                    my_longit = self.vehicle.location.global_relative_frame.lon
                    my_altit = self.vehicle.location.global_relative_frame.alt
                    my_loc = LocationGlobalRelative(my_latit, my_longit, my_altit)

                    target_myloc_bearing = get_bearing(my_loc, self.target_loc)
                    dist_target_myloc = get_distance_meters(my_loc, self.target_loc)
                    self.condition_yaw(target_myloc_bearing, relative=False, direction=True)
                    vel = self.kerr_velx * (dist_target_myloc - self.dist_min)
                    vel = saturate(vel, self.vel_x_min, self.vel_x_max, self.vel_zero)
                    vel_north = vel * math.cos(target_myloc_bearing * math.pi / 180)
                    vel_east = vel * math.sin(target_myloc_bearing * math.pi / 180)

                    self.send_ned_velocity(vel_north, vel_east, 0)
                    print "VNorth = %.2f m/sec" % (vel_north), "; VEast = %.2f m/sec" % (vel_east)

                    if vel == 0:
                        print "Balloon reached!"
                        print "CC State Machine Mode switching to: MISSION ACTION"
                        self.mode = 'MISSION ACTION'

            elif self.mode == 'MISSION ACTION':

                (locked, _, _, err_x_m, err_y_m, dist, psi_err, _) = self.get_input()[1:]
                print "Doing some ACTION..."
                time.sleep(20)

                my_latit = self.vehicle.location.global_relative_frame.lat
                my_longit = self.vehicle.location.global_relative_frame.lon
                my_altit = self.vehicle.location.global_relative_frame.alt + self.delta_alt
                my_loc = LocationGlobalRelative(my_latit, my_longit, my_altit)
                my_heading = self.vehicle.heading

                self.vehicle.simple_goto(my_loc)
                self.condition_yaw(my_heading, relative=False, direction=True)

                print "Changing altitude: "
                while True:
                    print self.vehicle.location.global_relative_frame.alt
                    if self.vehicle.location.global_relative_frame.alt >= my_loc.alt * 0.95:
                        print "New altitude reached"
                        break
                time.sleep(5)

                self.vehicle.mode = VehicleMode("AUTO")
                self.mode = 'MISSION SEARCHING'

            elif self.mode == 'BACK':

                if self.vehicle.location.global_relative_frame.alt < 1:
                    self.clear_mission()
                    print "Vehicle landed! Mission deleted"
                    self.mode = 'GROUND'
                    print "CC State Machine Mode switching to: GROUND"
            else:       # if manual control selected
                print "WARNING! Default branch in control FSM taken! This should never happen!"
                break
            time.sleep(0.5)
        self.vehicle.mode = VehicleMode("ALT_HOLD")
        print "Switched to ALT_HOLD mode"
        self.clear_mission()
        print "Mission cleared"
        self.close_vehicle()
        print "Vehicle closed"
