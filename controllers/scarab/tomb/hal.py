"""Hardware abstraction layer.

The mission code only talks to a RobotIO. WebotsIO wraps the real simulator;
tools/sim2d.py provides a light 2D stand-in with the same interface, so every
module can be developed and tested without opening Webots.
"""

import math

import numpy as np

from . import config


class RobotIO:
    """Interface the mission code relies on."""

    source = "robot"                 # tag used in log and figure names
    dt = 0.032                       # control period (s)
    scan_angles = np.linspace(config.LIDAR_FOV / 2, -config.LIDAR_FOV / 2, config.LIDAR_RAYS)

    def step(self):
        """Advance one control period. Returns False when the simulation ends."""
        raise NotImplementedError

    def time(self):
        raise NotImplementedError

    def pose(self):
        """(x, y, yaw) of the robot's axle centre in the world frame."""
        raise NotImplementedError

    def scan(self):
        """Lidar ranges (m), ordered left to right like scan_angles; inf = no return."""
        raise NotImplementedError

    def wheel_angles(self):
        """(left, right) wheel encoder angles (rad)."""
        raise NotImplementedError

    def set_wheel_speeds(self, left, right):
        raise NotImplementedError

    def place(self, x, y, yaw):
        """Move the robot to a start pose before the mission begins (experiments)."""
        raise NotImplementedError

    def keys(self):
        """Names of keys held this step, e.g. {'up', 'm'}."""
        return set()

    def show(self, rgb, lines=()):
        """Draw an RGB uint8 image (H x W x 3) plus text lines on the robot display."""

    def finish(self, quit_sim=False):
        self.set_wheel_speeds(0.0, 0.0)

    def drive(self, v, w):
        """Unicycle command (m/s, rad/s) -> wheel speeds, scaled to the motor limit."""
        half = config.AXLE_LENGTH / 2
        left = (v - w * half) / config.WHEEL_RADIUS
        right = (v + w * half) / config.WHEEL_RADIUS
        peak = max(abs(left), abs(right))
        if peak > config.MAX_WHEEL_SPEED:
            left *= config.MAX_WHEEL_SPEED / peak
            right *= config.MAX_WHEEL_SPEED / peak
        self.set_wheel_speeds(left, right)


class WebotsIO(RobotIO):
    """RobotIO backed by the Pioneer 3-DX in Webots."""

    source = "webots"

    def __init__(self):
        from controller import Display, Keyboard, Supervisor

        self._Display = Display
        self.robot = Supervisor()
        self.ts = int(self.robot.getBasicTimeStep())
        self.dt = self.ts / 1000.0

        self.lidar = self.robot.getDevice(config.LIDAR_NAME)
        self.lidar.enable(self.ts)
        n, fov = self.lidar.getHorizontalResolution(), self.lidar.getFov()
        # the range image runs from left to right (positive to negative angle)
        self.scan_angles = np.linspace(fov / 2, -fov / 2, n)

        self.gps = self.robot.getDevice("gps")
        self.gps.enable(self.ts)
        self.imu = self.robot.getDevice("imu")
        self.imu.enable(self.ts)

        self.motors = [self.robot.getDevice(n) for n in ("left wheel", "right wheel")]
        for m in self.motors:
            m.setPosition(float("inf"))
            m.setVelocity(0.0)
        self.encoders = [self.robot.getDevice(n) for n in ("left wheel sensor", "right wheel sensor")]
        for e in self.encoders:
            e.enable(self.ts)

        self.keyboard = self.robot.getKeyboard()
        self.keyboard.enable(self.ts)
        self._keymap = {Keyboard.UP: "up", Keyboard.DOWN: "down",
                        Keyboard.LEFT: "left", Keyboard.RIGHT: "right"}
        self.display = self.robot.getDevice("map")

    def step(self):
        return self.robot.step(self.ts) != -1

    def time(self):
        return self.robot.getTime()

    def pose(self):
        x, y, _ = self.gps.getValues()
        yaw = self.imu.getRollPitchYaw()[2]
        return x, y, yaw

    def scan(self):
        return np.asarray(self.lidar.getRangeImage(), dtype=float)

    def wheel_angles(self):
        return tuple(e.getValue() for e in self.encoders)

    def set_wheel_speeds(self, left, right):
        self.motors[0].setVelocity(left)
        self.motors[1].setVelocity(right)

    def place(self, x, y, yaw):
        # the controller runs as a supervisor, so it may move its own robot node
        node = self.robot.getSelf()
        node.getField("translation").setSFVec3f([x, y, 0.0975])
        node.getField("rotation").setSFRotation([0.0, 0.0, 1.0, yaw])
        node.resetPhysics()

    def keys(self):
        held = set()
        k = self.keyboard.getKey()
        while k != -1:
            base = k & 0xFFFF
            held.add(self._keymap.get(base) or chr(base).lower())
            k = self.keyboard.getKey()
        return held

    def show(self, rgb, lines=()):
        h, w, _ = rgb.shape
        img = self.display.imageNew(np.ascontiguousarray(rgb).tobytes(), self._Display.RGB, w, h)
        self.display.imagePaste(img, 0, 0, False)
        self.display.imageDelete(img)
        self.display.setColor(0x19191F)                # dark band so the text never covers the map
        self.display.fillRectangle(0, 0, w, 4 + 11 * len(lines))
        self.display.setColor(0xF0D890)
        for i, text in enumerate(lines):
            self.display.drawText(text, 4, 4 + 11 * i)

    def export_view(self, path):
        """Save the 3D view as an image (for report figures)."""
        self.robot.exportImage(path, 90)

    def finish(self, quit_sim=False):
        super().finish()
        if quit_sim:
            self.robot.simulationQuit(0)
        # otherwise keep the world alive so the final map stays on screen
        while self.robot.step(self.ts) != -1:
            pass


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi
