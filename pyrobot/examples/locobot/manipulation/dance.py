
from pyrobot import Robot
import threading

def arm_dance(arm):
    target_joints = [
        [.5, .5, -1, 0, 0],
        [.25, -.5, 0, 0, 0],
        [-.5, .5, -1, 0, 0],
        [-.25, -.5, 0, 0, 0]
    ]
    print("Arm is Dancing")
    for joint in target_joints:
        arm.set_joint_positions(joint, plan=True)
    print("Arm Dance Complete")

def base_dance(base):
    print("Base is Dancing")
    base.set_vel(0, 5, 1.5)
    base.set_vel(0, 0, .5)
    base.set_vel(0, -5, 1.5)
    base.set_vel(0, 0, .5)
    base.set_vel(0, -5, 1.5)
    base.set_vel(0, 0, .5)
    base.set_vel(0, 5, 1.5)
    base.set_vel(0, 0, .5)
    print("Base Dance Complete")

def dance():
    config = dict(moveit_planner='ESTkConfigDefault')
    bot = Robot('locobot', arm_config=config)
    print(bot.base.configs.BASE.MAX_ABS_TURN_SPEED)

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
