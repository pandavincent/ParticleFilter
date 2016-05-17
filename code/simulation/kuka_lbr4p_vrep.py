"""
Module to control the KUKA LBR4+ in V-REP.
"""

from vrep import vrep as vrep


class KukaLBR4PlusVrep:
    """Class to control the KUKA LBR4+ Manipulator in V-REP.
    """

    def __init__(self, client_id):
        self._clientID = client_id
        # query joints
        self._joints = []
        for i in range(1, 8):
            rc, joint = vrep.simxGetObjectHandle(
                self._clientID, "LBR4p_joint{}".format(i),
                vrep.simx_opmode_oneshot_wait)
            self._joints.append(joint)

    def go_to(self, joint, angle):
        """Go to specified target angle.

        Args:
            joint (int): number of joint to change (0 to 7)
            angle (float): radians
        """
        vrep.simxSetJointTargetPosition(self._clientID, self._joints[joint], angle,
                                        vrep.simx_opmode_oneshot_wait)

    def enable_painting(self):
        """Enable spray painting end-effector.
        """
        vrep.simxSetIntegerSignal(self._clientID, "paintingEnabled", 1, vrep.simx_opmode_oneshot_wait)

    def disable_painting(self):
        """Disable spray painting end-effector.
        """
        vrep.simxSetIntegerSignal(self._clientID, "paintingEnabled", 0, vrep.simx_opmode_oneshot_wait)

    def set_color(self, r, g, b):
        """Set spray painting color (RGB).

        Args:
            r (float): red component (0 to 1)
            g (float): green component (0 to 1)
            b (float): blue component (0 to 1)
        """
        vrep.simxSetFloatSignal(self._clientID, "paintingColorR", r, vrep.simx_opmode_oneshot_wait)
        vrep.simxSetFloatSignal(self._clientID, "paintingColorG", g, vrep.simx_opmode_oneshot_wait)
        vrep.simxSetFloatSignal(self._clientID, "paintingColorB", b, vrep.simx_opmode_oneshot_wait)
        vrep.simxSetIntegerSignal(self._clientID, "paintingUpdate", 1, vrep.simx_opmode_oneshot_wait)
