from hub import port
import runloop
import time
import motor_pair
import color_matrix
import color
import motor
from hub import motion_sensor 
import hub

# gyro straight v1
async def gyro_straight(degrees, angle, correction, speed):
    motor.reset_relative_position(port.F, 0)

    while motor.relative_position(port.F) < degrees:
        yaw = motion_sensor.tilt_angles()[0] *-0.1
        print("Yaw: " + str(yaw))
        if(yaw > angle + 60):
            print("Skipped Yaw Value: " + str(yaw))
            motor_pair.move(motor_pair.PAIR_1, 0, velocity = speed)
            await runloop.sleep_ms(10)
        else:
            error = yaw - angle
            print("E: " + str(error))
            # correction is an integer which is the negative of the error
            proportion = int(error * correction)
            if proportion < -50 or proportion > 50:
                print("Skipped Yaw Value steering too high: " + str(proportion))
                motor_pair.move(motor_pair.PAIR_1, 0, velocity = speed)
            else:
                print("PE: " + str(proportion))
                # apply steering to correct the error
                motor_pair.move(motor_pair.PAIR_1, proportion, velocity = speed)
                await runloop.sleep_ms(10)
    motor_pair.stop(motor_pair.PAIR_1, stop = motor.HOLD)

# backward gyro straight v2
async def gyro_straight_back(degrees, angle, correction, speed):
    motor.reset_relative_position(port.B, 0)

    while motor.relative_position(port.B) < degrees:
        yaw = motion_sensor.tilt_angles()[0] *-0.1
        print(str(motor.relative_position(port.B)))
        if(yaw > angle + 60):
            #print("Skipped Yaw Value: " + str(yaw))
            motor_pair.move(motor_pair.PAIR_1, 0, velocity = speed)
            await runloop.sleep_ms(10)
        else:
            error = angle - yaw
            #print("E: " + str(error))
            # correction is an integer which is the negative of the error
            proportion = int(error * correction)
            if proportion < -50 or proportion > 50:
                motor_pair.move(motor_pair.PAIR_1, 0, velocity = speed)
            else:
                #print("PE: " + str(proportion))
                # apply steering to correct the error
                motor_pair.move(motor_pair.PAIR_1, proportion, velocity = speed)
                await runloop.sleep_ms(10)
    motor_pair.stop(motor_pair.PAIR_1, stop = motor.HOLD)

# Gyro straight v3
# For gyro tank straight use tuned values
# for speed = 250, -0.8
# for speed = 360, -1.0
# for speed = 400, -1.8
# for speed = 500, -2.0
# for speed = 550, -2.2
# for backwards, send degrees in negative and backwards = true. speed can be positive value
async def gyro_straight_tank_gyro_error(degrees, angle, correction, speed, backwards = False):
    motor.reset_relative_position(port.F, 0)
    count = 0
    while (True):
        count = count + 1
        if(backwards):
            if(motor.relative_position(port.F) < degrees):
                break
        else:
            if(motor.relative_position(port.F) > degrees):
                break
        yaw = motion_sensor.tilt_angles()[0] *-0.1
        print("Yaw: " + str(yaw))
        if(yaw > angle + 60):
            print("Skipped Yaw Value(" + str(count) + "): " + str(yaw))
            motor_pair.move_tank(motor_pair.PAIR_1, speed, speed)
            await runloop.sleep_ms(10)
        else:
            error = yaw - angle
            print("E: " + str(error))
            # correction is an integer which is the negative of the error
            proportion = int(error * correction)
            if proportion < -50 or proportion > 50:
                print("Skipped Yaw Value steering too high(" + str(count) + "): " + str(proportion))
                await runloop.sleep_ms(10)
            else:
                print("speed delta: " + str(proportion))
                print("left velocity : " + str(speed+proportion))
                print("right velocity: " + str(speed-proportion))
                if(backwards):
                    # apply steering to correct the error
                    motor_pair.move_tank(motor_pair.PAIR_1, -(speed-proportion), -(speed+proportion))
                else:
                    # apply steering to correct the error
                    motor_pair.move_tank(motor_pair.PAIR_1, speed+proportion, speed-proportion)
                await runloop.sleep_ms(10)
    motor_pair.stop(motor_pair.PAIR_1, stop = motor.HOLD)

# Gyro straight v4
# removed reset_yaw_face and workarounds for gyro
# For gyro tank straight use tuned values
# with wall left and high, use higher values
# for speed = 250, -0.8
# for speed = 360, -1.0
# for speed = 400, -1.8
# for speed = 500, -2.0
# for speed = 550, -2.2
# for backwards, send degrees in negative and backwards = true. speed can be positive value
async def gyro_straight_tank(degrees, angle, correction, speed, backwards = False, log = False, delay=15):
    motor.reset_relative_position(port.F, 0)
    count = 0
    while (True):
        count = count + 1
        if(backwards):
            if(motor.relative_position(port.F) < degrees):
                break
        else:
            if(motor.relative_position(port.F) > degrees):
                break
        yaw = motion_sensor.tilt_angles()[0] *-0.1
        if(log):
            print("Yaw: " + str(yaw))
        if(yaw > angle + 60):
            print("Skipped Yaw Value(" + str(count) + "): " + str(yaw))
        error = yaw - angle
        if(log):
            print("E: " + str(error))
        # correction is an integer which is the negative of the error
        proportion = int(error * correction)
        if proportion < -80 or proportion > 80:
            print("Steering too high(" + str(count) + "): " + str(proportion))
        if(log):
            print("speed delta: " + str(proportion))
        if(backwards):
            # apply steering to correct the error
            #print("backwards left velocity : " + str(-(speed-proportion)))
            #print("backwards right velocity: " + str(-(speed+proportion)))
            motor_pair.move_tank(motor_pair.PAIR_1, -(speed-proportion), -(speed+proportion))
        else:
            # apply steering to correct the error
            #print("forwards left velocity : " + str(speed+proportion))
            #print("forwards right velocity: " + str(speed-proportion))
            motor_pair.move_tank(motor_pair.PAIR_1, speed+proportion, speed-proportion)
        await runloop.sleep_ms(delay)
    motor_pair.stop(motor_pair.PAIR_1, stop = motor.HOLD)

# Gyro straight v5 decelrated gyro straight
# removed reset_yaw_face and workarounds for gyro
# For gyro tank straight use tuned values
# with wall left and high, use higher values
# for speed = 250, -0.8
# for speed = 360, -1.0
# for speed = 400, -1.8
# for speed = 500, -2.0
# for speed = 550, -2.2
# for backwards, send degrees in negative and backwards = true. speed can be positive value
async def gyro_straight_decel_tank(degrees, angle, correction, speed, backwards = False, log = False, delay=15, decel_to_speed=50, decel_at_distance=50):
    motor.reset_relative_position(port.F, 0)
    count = 0
    while (True):
        count = count + 1
        if(backwards):
            if(motor.relative_position(port.F) < degrees):
                break
        else:
            if(motor.relative_position(port.F) > degrees):
                break
        yaw = motion_sensor.tilt_angles()[0] *-0.1
        if(log):
            print("Yaw: " + str(yaw))
        if(yaw > angle + 60):
            print("Skipped Yaw Value(" + str(count) + "): " + str(yaw))
        error = yaw - angle
        if(log):
            print("E: " + str(error))
        # correction is an integer which is the negative of the error
        proportion = int(error * correction)
        if proportion < -80 or proportion > 80:
            print("Steering too high(" + str(count) + "): " + str(proportion))
        if(log):
            print("speed delta: " + str(proportion))
        decel_distance = abs(motor.relative_position(port.F) - degrees)
        # if less than decel_at_distance degrees from target then slow down at decel_speed but keep gyro straight angle
        if(decel_distance > decel_at_distance):
            if(backwards):
                # apply steering to correct the error
                #print("backwards left velocity : " + str(-(speed-proportion)))
                #print("backwards right velocity: " + str(-(speed+proportion)))
                motor_pair.move_tank(motor_pair.PAIR_1, -(speed-proportion), -(speed+proportion))
            else:
                # apply steering to correct the error
                #print("forwards left velocity : " + str(speed+proportion))
                #print("forwards right velocity: " + str(speed-proportion))
                motor_pair.move_tank(motor_pair.PAIR_1, speed+proportion, speed-proportion)
        else:
            decel_speed = decel_to_speed
            if(backwards):
                # apply steering to correct the error
                #print("backwards left velocity : " + str(-(speed-proportion)))
                #print("backwards right velocity: " + str(-(speed+proportion)))
                motor_pair.move_tank(motor_pair.PAIR_1, -(decel_speed-proportion), -(decel_speed+proportion))
            else:
                # apply steering to correct the error
                #print("forwards left velocity : " + str(speed+proportion))
                #print("forwards right velocity: " + str(speed-proportion))
                motor_pair.move_tank(motor_pair.PAIR_1, decel_speed+proportion, decel_speed-proportion)
        await runloop.sleep_ms(delay)
    motor_pair.stop(motor_pair.PAIR_1, stop = motor.HOLD)


# Normal gyro turn version 2
# for negative turn, use only negative yaw_angle and same speed (no negative speed)
# for direction send left or right
async def gyro_turn(yaw_angle, direction, speed):
    if direction == "right":
        if yaw_angle < 0:
            while motion_sensor.tilt_angles()[0]* -0.1 < yaw_angle:
                motor_pair.move_tank(motor_pair.PAIR_1, speed, -speed)
            motor_pair.stop(motor_pair.PAIR_1)
        elif yaw_angle >= 0:
            while motion_sensor.tilt_angles()[0]* -0.1 < yaw_angle:
                motor_pair.move_tank(motor_pair.PAIR_1, speed, -speed)
            motor_pair.stop(motor_pair.PAIR_1)
    elif direction == "left":
        if yaw_angle < 0:
            while motion_sensor.tilt_angles()[0]* -0.1 > yaw_angle:
                motor_pair.move_tank(motor_pair.PAIR_1, -speed, speed)
            motor_pair.stop(motor_pair.PAIR_1)
        if yaw_angle >= 0:
            while motion_sensor.tilt_angles()[0]* -0.1 > yaw_angle:
                motor_pair.move_tank(motor_pair.PAIR_1, -speed, speed)
            motor_pair.stop(motor_pair.PAIR_1)

# Decel gyro turn v4
# for negative turn, use negative yaw_angle and negative speed
# for direction, send left or right
async def gyro_turn_decel(yaw_angle, direction, additional_speed):
    if direction == "right":
        if yaw_angle >= 0:
            while motion_sensor.tilt_angles()[0]* -0.1 < yaw_angle:
                delta = int(yaw_angle - motion_sensor.tilt_angles()[0]*-0.1)
                #print("delta: " + str(delta))
                speed = delta + additional_speed
                #print("turn speed: " + str(speed))
                motor_pair.move_tank(motor_pair.PAIR_1, speed, -speed)
            motor_pair.stop(motor_pair.PAIR_1, stop = motor.HOLD)
        elif yaw_angle < 0:
            while motion_sensor.tilt_angles()[0]* -0.1 < yaw_angle:
                speed = int(yaw_angle - motion_sensor.tilt_angles()[0]*-0.1 ) + additional_speed
                #print("turn speed: " + str(speed))
                motor_pair.move_tank(motor_pair.PAIR_1, speed, -speed)
            motor_pair.stop(motor_pair.PAIR_1, stop = motor.HOLD)
    elif direction == "left":
        if yaw_angle < 0:
            while motion_sensor.tilt_angles()[0]* -0.1 > yaw_angle:
                speed = int(yaw_angle - motion_sensor.tilt_angles()[0]*-0.1 ) + additional_speed
                #print("turn speed: " + str(speed))
                motor_pair.move_tank(motor_pair.PAIR_1, speed, -speed)
            motor_pair.stop(motor_pair.PAIR_1, stop = motor.HOLD)
        if yaw_angle >= 0:
            while motion_sensor.tilt_angles()[0]* -0.1 > yaw_angle:
                speed = int(yaw_angle - motion_sensor.tilt_angles()[0]*-0.1 ) + additional_speed
                #print("turn speed: " + str(speed))
                motor_pair.move_tank(motor_pair.PAIR_1, speed, -speed)
            motor_pair.stop(motor_pair.PAIR_1, stop = motor.HOLD)

# Decel gyro turn v4
# for negative turn, use negative yaw_angle and negative speed
# for direction, send left or right
async def gyro_turn_decel_pivot_on_left(yaw_angle, direction, additional_speed):
    if direction == "right":
        if yaw_angle >= 0:
            while motion_sensor.tilt_angles()[0]* -0.1 < yaw_angle:
                delta = int(yaw_angle - motion_sensor.tilt_angles()[0]*-0.1)
                #print("delta: " + str(delta))
                speed = delta + additional_speed
                #print("turn speed: " + str(speed))
                motor_pair.move_tank(motor_pair.PAIR_1, speed, 0)
            motor_pair.stop(motor_pair.PAIR_1, stop = motor.HOLD)
        elif yaw_angle < 0:
            while motion_sensor.tilt_angles()[0]* -0.1 < yaw_angle:
                speed = int(yaw_angle - motion_sensor.tilt_angles()[0]*-0.1 ) + additional_speed
                #print("turn speed: " + str(speed))
                motor_pair.move_tank(motor_pair.PAIR_1, speed, 0)
            motor_pair.stop(motor_pair.PAIR_1, stop = motor.HOLD)
    elif direction == "left":
        if yaw_angle < 0:
            while motion_sensor.tilt_angles()[0]* -0.1 > yaw_angle:
                speed = int(yaw_angle - motion_sensor.tilt_angles()[0]*-0.1 ) + additional_speed
                #print("turn speed: " + str(speed))
                motor_pair.move_tank(motor_pair.PAIR_1, speed, 0)
            motor_pair.stop(motor_pair.PAIR_1, stop = motor.HOLD)
        if yaw_angle >= 0:
            while motion_sensor.tilt_angles()[0]* -0.1 > yaw_angle:
                speed = int(yaw_angle - motion_sensor.tilt_angles()[0]*-0.1 ) + additional_speed
                #print("turn speed: " + str(speed))
                motor_pair.move_tank(motor_pair.PAIR_1, speed, 0)
            motor_pair.stop(motor_pair.PAIR_1, stop = motor.HOLD)

# Decel gyro turn v4
# for negative turn, use negative yaw_angle and negative speed
# for direction, send left or right
async def gyro_turn_decel_pivot_on_right(yaw_angle, direction, additional_speed):
    if direction == "right":
        if yaw_angle >= 0:
            while motion_sensor.tilt_angles()[0]* -0.1 < yaw_angle:
                delta = int(yaw_angle - motion_sensor.tilt_angles()[0]*-0.1)
                #print("delta: " + str(delta))
                speed = delta + additional_speed
                #print("turn speed: " + str(speed))
                motor_pair.move_tank(motor_pair.PAIR_1, 0, -speed)
            motor_pair.stop(motor_pair.PAIR_1, stop = motor.HOLD)
        elif yaw_angle < 0:
            while motion_sensor.tilt_angles()[0]* -0.1 < yaw_angle:
                speed = int(yaw_angle - motion_sensor.tilt_angles()[0]*-0.1 ) + additional_speed
                #print("turn speed: " + str(speed))
                motor_pair.move_tank(motor_pair.PAIR_1, 0, -speed)
            motor_pair.stop(motor_pair.PAIR_1, stop = motor.HOLD)
    elif direction == "left":
        if yaw_angle < 0:
            while motion_sensor.tilt_angles()[0]* -0.1 > yaw_angle:
                speed = int(yaw_angle - motion_sensor.tilt_angles()[0]*-0.1 ) + additional_speed
                #print("turn speed: " + str(speed))
                motor_pair.move_tank(motor_pair.PAIR_1, 0, -speed)
            motor_pair.stop(motor_pair.PAIR_1, stop = motor.HOLD)
        if yaw_angle >= 0:
            while motion_sensor.tilt_angles()[0]* -0.1 > yaw_angle:
                speed = int(yaw_angle - motion_sensor.tilt_angles()[0]*-0.1 ) + additional_speed
                #print("turn speed: " + str(speed))
                motor_pair.move_tank(motor_pair.PAIR_1, 0, -speed)
            motor_pair.stop(motor_pair.PAIR_1, stop = motor.HOLD)



# we learnt that motor.run_for_degrees is async by default, so we need to await if we want to it one at a time
async def move_wall_horizontal(degrees, speed, hwait):#degrees from the center is 750 degrees to any of the, from one end to the other end is 1500
    if(hwait):
        await motor.run_for_degrees(port.E, degrees, speed)
    else:
        motor.run_for_degrees(port.E, degrees, speed)

# we learnt that motor.run_for_degrees is async by default, so we need to await if we want to it one at a time
async def move_wall_vertical(degrees, speed, vwait):#from the bottem the wall can go up is 1700 degrees
    if(vwait):
        await motor.run_for_degrees(port.C, degrees, speed)
    else:
        motor.run_for_degrees(port.C, degrees, speed)


#start position: 3rd line from the first thick black line on the right
#THE WALL GOES UP WITH - SPEED
async def main():
    motor_pair.pair(motor_pair.PAIR_1, port.B, port.F)
    # Reset the yaw angle and wait for it to stabilize
    #motion_sensor.set_yaw_face(motion_sensor.FRONT)
    motion_sensor.reset_yaw(0)
    await runloop.until(motion_sensor.stable)
    
    #going to the boat
    await move_wall_vertical(200, 1000, True)
    await gyro_straight_tank(460, 0, -2.2, 400)
    await runloop.sleep_ms(20)
    print("yaw to boat : " + str(motion_sensor.tilt_angles()[0] *-0.1))
    await move_wall_vertical(1500, 1000, True)
    
    #turn to do the boat
    await gyro_turn_decel(25, "right", 45)
    await gyro_straight_tank(-100, 25, -0.8, 250, True)
    
    #move wall to collect the items
    runloop.run(gyro_turn_decel(-15, "left", -35), move_wall_vertical(1675, -1000, True))
    await gyro_turn_decel(0, "right", 30)
    runloop.run(gyro_straight_tank(100, 0, -1.2, 360), move_wall_horizontal(500, -1000, True))
    await runloop.sleep_ms(50)
    await gyro_straight_tank(170, 0, -0.4, 180)
    await runloop.sleep_ms(25)
    await move_wall_horizontal(750, 1000, True)
    await gyro_straight_tank(200, 0, -0.4, 150)
    await runloop.sleep_ms(15)
    await move_wall_horizontal(250, -1000, True)
    
    #turn left to allign plankton
    await gyro_turn_decel(-88, "left", -10)
    
    #back up to collect plankton
    await gyro_straight_tank(-150, -88, -0.6, 200, True)
    await runloop.sleep_ms(20)
    
    #move forward to the angler fish
    await gyro_straight_decel_tank(600, -82, -2.2, 450)
    #print("yaw before 1st wait : " + str(motion_sensor.tilt_angles()[0] *-0.1))
    await runloop.sleep_ms(50) 
    
    #line up to angler fish
    #do angler fish
    runloop.run(gyro_straight_decel_tank(415, -85, -2.2, 450), move_wall_horizontal(500, -1000, True))
    #print("yaw before angler fish : " + str(motion_sensor.tilt_angles()[0] *-0.1))
    await runloop.sleep_ms(50)
    
    #get seebed sample
    await move_wall_horizontal(1100, 1000, True)
    await gyro_straight_tank(220, -85, -0.6, 200)
    await move_wall_horizontal(600, -1000, True)
    await gyro_turn_decel(-88, "left", -25)

    #start return
    await gyro_straight_tank(150, -88, -0.6, 200)
    #print("yaw before return : " + str(motion_sensor.tilt_angles()[0] *-0.1))
    await gyro_turn_decel(-95, "left", -25)
    await gyro_straight_decel_tank(250, -95, -2.0, 400)
    await gyro_turn_decel(-140, "left", -25)
    
    #back to home
    runloop.run(motor_pair.move_tank_for_degrees(motor_pair.PAIR_1, 930, 550, 550), move_wall_vertical(25, -1000, True))
    
runloop.run(main())
