# python -m Pyro4.naming -n <MYIP>

import Pyro4
from pyrobot import Robot
import numpy as np

MYIP = "192.168.0.124"
Pyro4.config.SERIALIZERS_ACCEPTED.add('pickle')


@Pyro4.expose
class RemoteLoCoRobot(object):
    def __init__(self):
        base_config_dict={'base_controller': 'proportional'} 
        self._robot = Robot("locobot",
                            use_base=True,
                            use_arm=True,
                            use_camera=True, 
                            base_config=base_config_dict)
        self._done = True

    def test_connection(self):
        print("Connected!!")  # should print on server terminal
        return "Connected!"  # should print on client terminal

    # Navigation wrapper
    @Pyro4.oneway
    def go_home(self):
        """Go to original point: x, y, yaw 0, 0, 0"""
        if self._done:
            self._done = False
            self._robot.base.go_to_absolute([0,0,0]) 
            self._done = True

    @Pyro4.oneway
    def go_to_absolute(self, xyt_position, use_map=False, close_loop=False, smooth=False):
        """Go to absolute position: x, y, yaw"""
        if self._done:
            self._done = False
            self._robot.base.go_to_absolute(xyt_position, use_map=use_map, close_loop=close_loop, smooth=smooth) 
            self._done = True

    @Pyro4.oneway
    def go_to_relative(self, xyt_position, use_map=False, close_loop=False, smooth=False):
        """Go to absolute position: x, y, yaw"""
        if self._done:
            self._done = False
            self._robot.base.go_to_relative(xyt_position, use_map=use_map, close_loop=close_loop, smooth=smooth) 
            self._done = True

    @Pyro4.oneway
    def stop(self):
        """stop"""
        self._robot.base.stop()

    @Pyro4.oneway
    def track_trajectory(self, states, controls=None, close_loop=False):
        if self._done:
            self._done = False
            self._robot.base.track_trajectory(states, controls=controls, close_loop=close_loop) 
            self._done = True

    def get_base_state(self, state_type):
        """Same as base.get_state"""
        return self._robot.base.get_state(state_type)

    # Manipulation wrapper
    @Pyro4.oneway
    def move_to_neutral(self):
        if self._done:
            self._done = False
            self._robot.arm.move_to_neutral()
            self._done = True

    @Pyro4.oneway
    def set_joint_positions(self, target_joint, plan=False):
        if self._done:
            self._done = False
            target_joint = np.array(target_joint)
            self._robot.arm.set_joint_positions(target_joint, plan=plan)
            self._done = True

    @Pyro4.oneway
    def set_joint_velocities(self, target_vels):
        if self._done:
            self._done = False
            target_vels = np.array(target_vels)
            self._robot.arm.set_joint_velocities(target_vels)
            self._done = True

    @Pyro4.oneway
    def set_ee_pose(self, position, orientation, plan=False):
        if self._done:
            self._done = False
            position = np.array(position)
            orientation = np.array(orientation)
            self._robot.arm.set_ee_pose(position, orientation, plan=plan)
            self._done = True

    @Pyro4.oneway
    def move_ee_xyz(self, displacement, eef_step=0.005, plan=False):
        if self._done:
            self._done = False
            displacement = np.array(displacement)
            self._robot.arm.move_ee_xyz(displacement, eef_step, plan=plan)
            self._done = True

    # Gripper wrapper
    @Pyro4.oneway
    def open_gripper(self):
        if self._done:
            self._done = False
            self._robot.gripper.open()
            self._done = True

    @Pyro4.oneway
    def close_gripper(self):
        if self._done:
            self._done = False
            self._robot.gripper.close()
            self._done = True

    def get_gripper_position(self):
        """Between 0 and 0.041.
        """
        return self._robot.gripper.gripper.get_position()

    def get_end_eff_pose(self):
        """Returns (position, rotation matrix, quaternion).
        """
        pos, rotmat, quat = self._robot.arm.pose_ee
        return pos.flatten().tolist(), rotmat.tolist(), quat.tolist()

    def get_joint_positions(self):
        return self._robot.arm.get_joint_angles().tolist()

    def get_joint_velocities(self):
        return self._robot.arm.get_joint_velocities().tolist()

    # Common wrapper
    def command_finished(self):
        return self._done

    def kill(self):
        """Kill any ongoing call to loop_angle_pub_cmd().
        """
        self._robot.arm.emergency_stop = True

    def unkill(self):
        """Unkill any ongoing call to loop_angle_pub_cmd().
        """
        self._robot.arm.emergency_stop = False

    # Camera wrapper
    def get_current_pcd(self, in_cam=True):
        """Return the point cloud at current time step (one frame only)"""
        pts, colors = self._robot.camera.get_current_pcd(in_cam=in_cam)
        return pts.tolist(), colors.tolist()

    def get_depth(self):
        depth = self._robot.camera.get_depth()
        if depth:
            print(depth.tolist())
            return depth.tolist()
        return None

    def get_intrinsics(self):
        intrinsics = self._robot.camera.get_intrinsics()
        if intrinsics:
            intrinsics = intrinsics.tolist()
            return intrinsics.tolist()
        return None

    def get_rgb(self):
        rgb = self._robot.camera.get_rgb()
        if rgb:
            return rgb.tolist()
        return None

    def get_rgb_depth(self):
        rgb, depth = self._robot.camera.get_rgb_depth()
        return rgb.tolist(), depth.tolist()

    def pix_to_3dpt(self, rs, cs, in_cam=False):
        pts, colors = self._robot.camera.pix_to_3dpt(rs, cs, in_cam=in_cam)
        return pts.tolist(), colors.tolist()

    # Camera pan wrapper
    def get_pan(self):
        return self._robot.camera.get_pan()

    def get_camera_state(self):
        return self._robot.camera.get_state()

    def get_tilt(self):
        return self._robot.camera.get_tilt()

    @Pyro4.oneway
    def reset(self):
        if self._done:
            self._done = False
            self._robot.camera.reset() 
            self._done = True

    @Pyro4.oneway
    def set_pan(self, pan, wait=True):
        if self._done:
            self._done = False
            self._robot.camera.set_pan(pan, wait=True) 
            self._done = True

    @Pyro4.oneway
    def set_pan_tilt(self, pan, tilt, wait=True):
        if self._done:
            self._done = False
            self._robot.camera.set_pan_tilt(pan, tilt, wait=True) 
            self._done = True

    @Pyro4.oneway
    def set_tilt(self, tilt, wait=True):
        if self._done:
            self._done = False
            self._robot.camera.set_tilt(tilt, wait=True) 
            self._done = True


with Pyro4.Daemon(MYIP) as daemon:
    robot = RemoteLoCoRobot()
    robot_uri = daemon.register(robot)
    with Pyro4.locateNS() as ns:
        ns.register("remotelocorobot", robot_uri)

    print("Server is started...")
    daemon.requestLoop()


# Below is client code to run in a separate Python shell...
# import Pyro4
# robot = Pyro4.Proxy("PYRONAME:remotelocorobot")
# robot.go_home()
