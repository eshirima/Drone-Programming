import time
import socket
import exceptions
import dronekit_sitl

from dronekit import connect, VehicleMode, APIException

class Copter:
    def __init__(self):
        self.__vehicle = None
        self.__call_backs = {}
        self.__launch_sequence_called = False
        self.__take_off_duration = 0  # used to estimate the landing time

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
            return False, 'No vehicle found. Try connecting one first.'

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
        return True, None

    def add_call_back(self, name, call_back):
        self.__call_backs[name] = call_back

    def remove_call_back(self, name, call_back):
        del self.__call_backs[name]
        self.__vehicle.remove_attribute_listener(name, call_back)

    def rise_to_height(self, height=20, hover_duration=10):
        if not self.__launch_sequence_called:
            return False, 'Initiate launch sequence first'

        self.__vehicle.simple_takeoff(height)

        start_time = time.time()

        while True:
            # print 'Copter Altitude', self.__vehicle.location.global_relative_frame.alt
            if self.__vehicle.location.global_relative_frame.alt == height:
                print '****** HEIGHT REACHED ******'
                self.__take_off_duration = time.time() - start_time
                break
            time.sleep(1)

        print '****** HOVERING FOR {} SECONDS'.format(hover_duration)

        # let the copter hover
        time.sleep(hover_duration)

        return True, None

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