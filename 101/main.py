# Author: Emil Shirima
# Date: Wednesday 07, February 2018
# Purpose: Homework 1 Solutions

import dronekit_sitl
import socket
import exceptions

from dronekit import connect, VehicleMode, LocationGlobal, APIException

# just for printing purposes
def separateResponse(isEnd=True):
    print '+' * 30 if isEnd else '*' * 30

def createSession(url, time_out=15, baud=115200):
    try:
        vehicle = connect(url, wait_ready=True, heartbeat_timeout=time_out, baud=baud)
        return vehicle, None

    except socket.error: return None, 'Server not found'
    except exceptions.OSError: return None, 'Serial not found'
    except APIException: return None, 'Timeout'
    except: return None, 'Something went wrong'

separateResponse(False)
print 'QUESTION 1 ANSWERS'
defaults = dronekit_sitl.start_default()
connection_string = defaults.connection_string()

result = createSession(connection_string, baud=128000)
vehicle = None

if result[0] is None: # session creation issue
    raise SystemExit(result[-1])

vehicle = result[0]

print vehicle.battery
print vehicle.gps_0
print vehicle.location.global_frame
print vehicle.location.local_frame
print vehicle.location.global_relative_frame
separateResponse()

separateResponse(False)
print 'QUESTION 2.4 & 2.5 ANSWERS'

for key in sorted(vehicle.__dict__):
    print key

separateResponse()

# QUESTION 2.6 ANSWERS
#
# vehicle.channels.overrides = {'2': 200, '4': 400}
# vehicle.gimbal.rotate(-90, 0, 0)
# new_loc = LocationGlobal(-34.364114, 149.166022, 30)
# vehicle.gimbal.target_location(new_loc)
# vehicle.home_location = new_loc
# vehicle.armed = True
# vehicle.mode = VehicleMode('GUIDED')
# vehicle.capabilities.set_actuator_target
# vehicle.capabilities.set_altitude_target_global_int
# vehicle.capabilities.set_attitude_target
# vehicle.capabilities.set_attitude_target_local_ned
# vehicle.capabilities.set_attitude_target_local_ned
# vehicle.airspeed = 10.0
# vehicle.groundspeed = 12.0

vehicle.close()
defaults.stop()