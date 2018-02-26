# Author: Emil Shirima
# Date: Monday 26th February 2018

from cop_main import *

def altitude_listener(self, _, value):
    print 'Copter Altitude: {}'.format(value.alt)

copter = Copter()

cop_object, error = copter.create_session()

if error is not None:
    raise SystemExit(error)

success, error = copter.init_launch_sequence()

cop_object.location.add_attribute_listener('global_relative_frame', altitude_listener)
copter.add_call_back('global_relative_frame', altitude_listener)
time.sleep(2)

if not success:
    raise SystemExit(error)

success, error = copter.rise_to_height()

if not success:
    raise SystemExit(error)

copter.shutdown()