from cop_main import *

def altitude_listener(self, _, value):
    print 'Copter Altitude: {}'.format(value.alt)

copter = Copter()

cop_object, error = copter.create_session()

if error is not None:
    raise SystemExit(error)

status = copter.init_launch_sequence()

cop_object.location.add_attribute_listener('global_relative_frame', altitude_listener)
copter.add_call_back('global_relative_frame', altitude_listener)
time.sleep(2)

if not status.success:
    raise SystemExit(status.error)

status = copter.rise_to_height()

if not status.success:
    raise SystemExit(status.error)

# change these parameters as desired
direction = Direction()
direction.north = 9
direction.south = -2
direction.east = 8
direction.west = -10
direction.up = -3.5
direction.down = .5

# just random flying routines
copter.set_ned_velocity(direction.south, 0, direction.up)
copter.set_ned_velocity(0, 0, 0, 1)

time.sleep(30)

copter.set_ned_velocity(0, direction.east, direction.down)
copter.set_ned_velocity(0, 0, 0, 1)

time.sleep(30)

copter.set_ned_velocity(direction.north, 0, 0)
copter.set_ned_velocity(0, 0, 0, 1)

time.sleep(30)

copter.set_ned_velocity(0, direction.east, 0)
copter.set_ned_velocity(0, 0, 0, 1)

time.sleep(30)

copter.set_ned_velocity(0, direction.west, direction.down)
copter.set_ned_velocity(0, 0, 0, 1)

time.sleep(30)

# if final_altitude != 0, then call copter.shutdown()
copter.return_to_launch_pad(altitude=0, final_altitude=0, loiter=5)
time.sleep(1000)

# copter.shutdown()