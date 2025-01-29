from hub import port
import runloop
import time
import motor_pair
import color_matrix
import color
import motor
from hub import motion_sensor
import hub

async def main():
    # write your code here
    motor_pair.pair(motor_pair.PAIR_1, port.B, port.F)
    await motor_pair.move_for_degrees(motor_pair.PAIR_1, 10000, 0, velocity=320)

runloop.run(main())
