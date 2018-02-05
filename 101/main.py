import dronekit_sitl
import dronekit
import socket
import exceptions

def createSession(url, time_out=15):
    try:
        vehicle = dronekit.connect(url, wait_ready=True, heartbeat_timeout=time_out)
        return vehicle, None

    except socket.error: return None, 'Server not found'
    except exceptions.OSError: return None, 'Serial not found'
    except dronekit.APIException: return None, 'Timeout'
    except: return None, 'Something went wrong'

defaults = dronekit_sitl.start_default()
connection_string = defaults.connection_string()

print connection_string

result = createSession(connection_string)
vehicle = None

if result[0] is None: # session creation issue
    raise SystemExit(result[-1])

vehicle = result[0]

print vehicle.battery
print vehicle.gps_0
print vehicle.location.global_frame
print vehicle.location.local_frame
print vehicle.location.global_relative_frame

vehicle.close()
defaults.stop()