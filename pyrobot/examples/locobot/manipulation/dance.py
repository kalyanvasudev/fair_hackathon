
from pyrobot import Robot
import threading
from time import sleep
import math as m

def arm_dance(arm):
    target_joints = [
        [0, 0, 0, 0, 0],
        [.25, -.5, 0, 0, 0],
        [-.5, .5, -1, 0, 0],
        [-.25, -.5, 0, 0, 0]
    ]
    print("Arm is Dancing")
    steps = 300
    for i in range(0, steps):
        joint = target_joints[0];

        if i > steps * 0.8:
            s = (steps - i) / (steps * 0.2)
        else:
            s = 1

        joint[0] = s*-0.75 * m.sin(0.5 * i/10.0 - m.pi/5)
        joint[1] = s*-0.6 * m.sin(i/10.0)
        joint[2] = s*0.75 * m.sin(i/10.0 + m.pi/5)
        joint[3] = s*1.5 * m.sin(i/10.0 + 2*m.pi/5)

        arm.set_joint_positions(joint, plan=False, wait=False)
        sleep(0.05)
    print("Arm Dance Complete")

def base_dance(base):
    print("Base is Dancing")
    print("Max Vel ", base.configs.BASE.MAX_ABS_TURN_SPEED)
    base.set_vel(0, 5, 1)
    base.set_vel(0, 0, .5)
    base.set_vel(0, -5, 1)
    base.set_vel(0, 0, .5)
    base.set_vel(0, -5, 1)
    base.set_vel(0, 0, .5)
    base.set_vel(0, 5, 1)
    base.set_vel(0, 0, .5)
    base.set_vel(0, 5, 1)
    base.set_vel(0, 0, .5)
    base.set_vel(0, -5, 1)
    base.set_vel(0, 0, .5)
    base.set_vel(0, -5, 1)
    base.set_vel(0, 0, .5)
    base.set_vel(0, 5, 1)
    base.set_vel(0, 0, .5)
    base.set_vel(0, 5, 1)
    base.set_vel(0, 0, .5)
    base.set_vel(0, -5, 1)
    base.set_vel(0, 0, .5)
    base.set_vel(0, -5, 1)
    base.set_vel(0, 0, .5)
    base.set_vel(0, 5, 1)
    base.set_vel(0, 0, .5)
    print("Base Dance Complete")

def dance():
    config = dict(moveit_planner='ESTkConfigDefault')
    bot = Robot('locobot', arm_config=config)
    print(bot.base.configs.BASE.MAX_ABS_TURN_SPEED)

    bot.arm.go_home()

    arm_dancer = threading.Thread(target=arm_dance, args=(bot.arm,))
    base_dancer = threading.Thread(target=base_dance, args=(bot.base,))

    arm_dancer.start()
    base_dancer.start()

    arm_dancer.join()
    base_dancer.join()

    bot.arm.go_home()

def main():
    dance()

if __name__ == "__main__":
    main()
