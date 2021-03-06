import time
import socket
import exceptions
import dronekit_sitl

from pymavlink import mavutil
from collections import namedtuple
from dronekit import connect, VehicleMode, APIException

class Direction(object):
    """
    Handles the orientation of the drone.
    """
    @property
    def north(self): return self._north

    @property
    def south(self): return self._south

    @property
    def east(self): return self._east

    @property
    def west(self): return self._west

    @property
    def up(self): return self._up

    @property
    def down(self): return self._down

    @north.setter
    def north(self, value):
        if value >= 0: self._north = value
        else: raise SystemError('To fly North, value has to be >= 0')

    @south.setter
    def south(self, value):
        if value <= 0: self._south = value
        else: raise SystemError('To fly South, value has to be <= 0')

    @east.setter
    def east(self, value):
        if value >= 0: self._east = value
        else: raise SystemError('To fly East, value has to be >= 0')

    @west.setter
    def west(self, value):
        if value <= 0: self._west = value
        else: raise SystemError('To fly WEst, value has to be <= 0')

    @up.setter
    def up(self, value):
        if value <= 0: self._up = value
        else: raise SystemError('To fly up, value has to be <= 0')

    @down.setter
    def down(self, value):
        if value >= 0: self._down = value
        else: raise SystemError('To fly down, value has to be >= 0')

class Copter:
    def __init__(self):
        self.__vehicle = None
        self.__call_backs = {}
        self.__launch_sequence_called = False
        self.__take_off_duration = 0  # used to estimate the landing time
        self.__status = namedtuple('status', ['success', 'error'])

    def create_session(self, url=None, time_out=15, baud=115200):

        if url is None:
            url = dronekit_sitl.start_default().connection_string()

        try:
            self.__vehicle = connect(url, wait_ready=True, heartbeat_timeout=time_out, baud=baud)
            return self.__vehicle, None

        except socket.error:
            return None, 'Server not found'
        except exceptions.OSError:
            return None, 'Serial not found'
        except APIException:
            return None, 'Timeout'
        except:
            return None, 'Something went wrong'

    def init_launch_sequence(self):

        # probably no session created
        if self.__vehicle is None:
            return self.__status(False, 'No vehicle found. Try connecting one first.')

        # don't try to arm until autopilot is ready
        while not self.__vehicle.is_armable:
            print '****** COPTER INITIALIZATION ******'
            time.sleep(2)

        self.__vehicle.mode = VehicleMode('GUIDED')
        self.__vehicle.armed = True
        # clear any prior missions; just in case
        self.__vehicle.flush()

        while not self.__vehicle.armed:
            print '****** ARMING COPTER ******'
            time.sleep(2)

        self.__launch_sequence_called = True
        return self.__status(True, None)

    def add_call_back(self, name, call_back):
        self.__call_backs[name] = call_back

    def remove_call_back(self, name, call_back):
        del self.__call_backs[name]
        self.__vehicle.remove_attribute_listener(name, call_back)

    def rise_to_height(self, height=20, hover_duration=10):
        if not self.__launch_sequence_called:
            return self.__status(False, 'Initiate launch sequence first')

        self.__vehicle.simple_takeoff(height)

        start_time = time.time()

        while True:
            if self.__vehicle.location.global_relative_frame.alt == height:
                print '****** HEIGHT REACHED ******'
                self.__take_off_duration = time.time() - start_time
                break
            time.sleep(1)

        print '****** HOVERING FOR {} SECONDS'.format(hover_duration)

        # let the copter hover
        time.sleep(hover_duration)
        return self.__status(True, None)

    def shutdown(self):

        # land the copter first then some Spring cleaning
        if not self.__vehicle.mode == VehicleMode('LAND'):
            print '****** LANDING COPTER ******'
            self.__vehicle.mode = VehicleMode('LAND')
            time.sleep(self.__take_off_duration)

        for key, value in self.__call_backs.iteritems():
            self.__vehicle.remove_attribute_listener(key, value)

        self.__vehicle.flush()
        self.__vehicle.close()
        self.__launch_sequence_called = False
        print '****** COPTER LANDED AND TURNED OFF ******'

    def return_to_launch_pad(self, altitude=0, final_altitude=0, loiter=5):
        """

        :param altitude: Minimum altitude; in cm, the copter will move to before returning to launch
        :param final_altitude: Altitude copter will move to at the final stage of RTL
        :param loiter: Time; in seconds, to hover above "Home" before beginning final descent
        :return: None
        """
        #TODO: Find a better way of checking if drone is flying? Probably check if its armed instead?
        if self.__vehicle.mode == VehicleMode('GUIDED'):
            self.__vehicle.parameters['RTL_ALT'] = altitude
            self.__vehicle.parameters['RTL_ALT_FINAL'] = final_altitude

            loiter_milliseconds = loiter * 1000
            if loiter_milliseconds > 60000: loiter_milliseconds = 60000

            self.__vehicle.parameters['RTL_LOIT_TIME'] = loiter_milliseconds

            self.__vehicle.mode = VehicleMode('RTL')


    def set_ned_velocity(self, x, y, z, duration=20):
        """
        Change the North-East-Down velocity of the copter.
        NED is a geographical coordinate system for representing
        state vectors that is commonly used in aviation.
        More info: https://en.wikipedia.org/wiki/North_east_down

        :param x: velocity along the x-axis
        :param y: velocity along the y-axis
        :param z: velocity along the z-axis
        :param duration: how long should it maintain this course?
        :return: None
        """
        path_info = self.__vehicle.message_factory.set_position_target_local_ned_encode(
            0,  # time_boot_ms (not used)
            0, 0,  # target system, target component
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,  # frame
            0b0000111111000111,  # type_mask (only speeds enabled)
            0, 0, 0,  # x, y, z positions (not used)
            x, y, z,  # x, y, z velocity in m/s
            0, 0, 0,  # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
            0, 0)  # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)

        # send the message every second
        for _ in range(0, duration):
            self.__vehicle.send_mavlink(path_info)
            time.sleep(1)