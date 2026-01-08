"""
3D OpenGL Renderer for IMU Visualization

Renders a 3D arrow/aircraft shape that rotates based on orientation,
along with a compass rose, grid floor, and axis indicators.
"""

import math
import pygame
from pygame.locals import *
from OpenGL.GL import *
from OpenGL.GLU import *

from sensor import Orientation, Quaternion


class Colors:
    """Color definitions (RGB 0-1)."""
    WHITE = (1.0, 1.0, 1.0)
    BLACK = (0.0, 0.0, 0.0)
    RED = (1.0, 0.2, 0.2)
    GREEN = (0.2, 1.0, 0.2)
    BLUE = (0.3, 0.5, 1.0)
    YELLOW = (1.0, 1.0, 0.2)
    CYAN = (0.2, 1.0, 1.0)
    ORANGE = (1.0, 0.6, 0.1)
    GRAY = (0.5, 0.5, 0.5)
    DARK_GRAY = (0.2, 0.2, 0.2)
    GRID = (0.3, 0.3, 0.4)


class IMURenderer:
    """OpenGL renderer for 3D IMU visualization."""

    def __init__(self, width: int = 1024, height: int = 768):
        self.width = width
        self.height = height
        # Default orientation: identity quaternion (no rotation)
        default_quat = Quaternion(1, 0, 0, 0)
        self.orientation = Orientation.from_quaternion(default_quat)
        self.calibration = (0, 0, 0, 0)
        self.font = None
        self.running = True

    def init(self):
        """Initialize pygame and OpenGL."""
        pygame.init()
        pygame.display.set_mode((self.width, self.height), DOUBLEBUF | OPENGL)
        pygame.display.set_caption("BNO055 IMU Visualizer")

        # OpenGL setup
        glEnable(GL_DEPTH_TEST)
        glEnable(GL_BLEND)
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
        glClearColor(0.1, 0.1, 0.15, 1.0)

        # Lighting
        glEnable(GL_LIGHTING)
        glEnable(GL_LIGHT0)
        glLightfv(GL_LIGHT0, GL_POSITION, (5, 10, 5, 1))
        glLightfv(GL_LIGHT0, GL_AMBIENT, (0.3, 0.3, 0.3, 1))
        glLightfv(GL_LIGHT0, GL_DIFFUSE, (0.8, 0.8, 0.8, 1))

        glEnable(GL_COLOR_MATERIAL)
        glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE)

        # Font for text overlay
        self.font = pygame.font.SysFont('monospace', 20)

    def set_perspective(self):
        """Set up 3D perspective projection."""
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        gluPerspective(45, self.width / self.height, 0.1, 100.0)
        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()
        gluLookAt(6, 4, 6,  # eye position
                  0, 0, 0,   # look at
                  0, 1, 0)   # up vector

    def draw_grid(self, size: int = 10, divisions: int = 20):
        """Draw a reference grid on the XZ plane."""
        glDisable(GL_LIGHTING)
        glColor3f(*Colors.GRID)
        glBegin(GL_LINES)

        step = size / divisions
        half = size / 2

        for i in range(divisions + 1):
            pos = -half + i * step
            # Lines parallel to X axis
            glVertex3f(-half, 0, pos)
            glVertex3f(half, 0, pos)
            # Lines parallel to Z axis
            glVertex3f(pos, 0, -half)
            glVertex3f(pos, 0, half)

        glEnd()
        glEnable(GL_LIGHTING)

    def draw_axes(self, length: float = 3.0):
        """Draw world coordinate axes."""
        glDisable(GL_LIGHTING)
        glLineWidth(2.0)
        glBegin(GL_LINES)

        # X axis - Red
        glColor3f(*Colors.RED)
        glVertex3f(0, 0, 0)
        glVertex3f(length, 0, 0)

        # Y axis - Green
        glColor3f(*Colors.GREEN)
        glVertex3f(0, 0, 0)
        glVertex3f(0, length, 0)

        # Z axis - Blue
        glColor3f(*Colors.BLUE)
        glVertex3f(0, 0, 0)
        glVertex3f(0, 0, length)

        glEnd()
        glLineWidth(1.0)
        glEnable(GL_LIGHTING)

    def draw_arrow_3d(self):
        """Draw a 3D arrow/aircraft shape representing the sensor orientation."""
        # Arrow body (fuselage)
        glColor3f(*Colors.ORANGE)

        # Main body - elongated shape pointing in +X direction
        glPushMatrix()
        glScalef(2.0, 0.3, 0.3)
        self._draw_cube()
        glPopMatrix()

        # Nose cone
        glPushMatrix()
        glTranslatef(1.2, 0, 0)
        glRotatef(90, 0, 1, 0)
        self._draw_cone(0.3, 0.6, 16)
        glPopMatrix()

        # Tail fin (vertical)
        glColor3f(*Colors.RED)
        glPushMatrix()
        glTranslatef(-0.8, 0.4, 0)
        glScalef(0.4, 0.5, 0.05)
        self._draw_cube()
        glPopMatrix()

        # Wings
        glColor3f(*Colors.CYAN)
        glPushMatrix()
        glTranslatef(0, 0, 0)
        glScalef(0.5, 0.05, 1.5)
        self._draw_cube()
        glPopMatrix()

        # Horizontal stabilizers
        glColor3f(*Colors.CYAN)
        glPushMatrix()
        glTranslatef(-0.8, 0, 0)
        glScalef(0.3, 0.05, 0.6)
        self._draw_cube()
        glPopMatrix()

        # Direction indicator arrow on top
        glColor3f(*Colors.YELLOW)
        glPushMatrix()
        glTranslatef(0.5, 0.25, 0)
        glRotatef(90, 0, 1, 0)
        self._draw_cone(0.1, 0.4, 8)
        glPopMatrix()

    def _draw_cube(self):
        """Draw a unit cube centered at origin."""
        vertices = [
            (-0.5, -0.5, -0.5), (0.5, -0.5, -0.5),
            (0.5, 0.5, -0.5), (-0.5, 0.5, -0.5),
            (-0.5, -0.5, 0.5), (0.5, -0.5, 0.5),
            (0.5, 0.5, 0.5), (-0.5, 0.5, 0.5)
        ]
        faces = [
            (0, 1, 2, 3), (4, 7, 6, 5),  # front, back
            (0, 4, 5, 1), (2, 6, 7, 3),  # bottom, top
            (0, 3, 7, 4), (1, 5, 6, 2)   # left, right
        ]
        normals = [
            (0, 0, -1), (0, 0, 1),
            (0, -1, 0), (0, 1, 0),
            (-1, 0, 0), (1, 0, 0)
        ]

        glBegin(GL_QUADS)
        for i, face in enumerate(faces):
            glNormal3f(*normals[i])
            for vi in face:
                glVertex3f(*vertices[vi])
        glEnd()

    def _draw_cone(self, radius: float, height: float, segments: int):
        """Draw a cone pointing in +Z direction."""
        glBegin(GL_TRIANGLE_FAN)
        glNormal3f(0, 0, 1)
        glVertex3f(0, 0, height)

        for i in range(segments + 1):
            angle = 2 * math.pi * i / segments
            x = radius * math.cos(angle)
            y = radius * math.sin(angle)
            glNormal3f(x, y, 0.5)
            glVertex3f(x, y, 0)
        glEnd()

        # Base
        glBegin(GL_TRIANGLE_FAN)
        glNormal3f(0, 0, -1)
        glVertex3f(0, 0, 0)
        for i in range(segments, -1, -1):
            angle = 2 * math.pi * i / segments
            glVertex3f(radius * math.cos(angle), radius * math.sin(angle), 0)
        glEnd()

    def draw_compass(self):
        """Draw a compass rose on the ground plane."""
        glDisable(GL_LIGHTING)
        radius = 4.0
        y = 0.01  # Slightly above grid

        # Compass circle
        glColor3f(*Colors.GRAY)
        glBegin(GL_LINE_LOOP)
        for i in range(64):
            angle = 2 * math.pi * i / 64
            glVertex3f(radius * math.cos(angle), y, radius * math.sin(angle))
        glEnd()

        # Cardinal direction markers
        directions = [
            (0, 'N', Colors.RED),
            (90, 'E', Colors.WHITE),
            (180, 'S', Colors.WHITE),
            (270, 'W', Colors.WHITE)
        ]

        glLineWidth(2.0)
        for angle_deg, label, color in directions:
            angle = math.radians(-angle_deg + 90)  # Convert to math convention
            x = radius * math.cos(angle)
            z = radius * math.sin(angle)

            glColor3f(*color)
            glBegin(GL_LINES)
            glVertex3f(x * 0.85, y, z * 0.85)
            glVertex3f(x, y, z)
            glEnd()

        # Tick marks every 30 degrees
        glColor3f(*Colors.GRAY)
        glLineWidth(1.0)
        for deg in range(0, 360, 30):
            if deg % 90 != 0:
                angle = math.radians(-deg + 90)
                x = math.cos(angle)
                z = math.sin(angle)
                glBegin(GL_LINES)
                glVertex3f(x * radius * 0.9, y, z * radius * 0.9)
                glVertex3f(x * radius, y, z * radius)
                glEnd()

        glEnable(GL_LIGHTING)

    def draw_heading_indicator(self):
        """Draw a line showing current heading on compass."""
        glDisable(GL_LIGHTING)
        glColor3f(*Colors.YELLOW)
        glLineWidth(3.0)

        angle = math.radians(-self.orientation.heading + 90)
        length = 4.5

        glBegin(GL_LINES)
        glVertex3f(0, 0.02, 0)
        glVertex3f(length * math.cos(angle), 0.02, length * math.sin(angle))
        glEnd()

        glLineWidth(1.0)
        glEnable(GL_LIGHTING)

    def draw_text_overlay(self):
        """Draw 2D text overlay with orientation data."""
        # Switch to 2D orthographic projection
        glDisable(GL_LIGHTING)
        glDisable(GL_DEPTH_TEST)

        glMatrixMode(GL_PROJECTION)
        glPushMatrix()
        glLoadIdentity()
        glOrtho(0, self.width, self.height, 0, -1, 1)
        glMatrixMode(GL_MODELVIEW)
        glPushMatrix()
        glLoadIdentity()

        # Render text
        lines = [
            f"Heading: {self.orientation.heading:7.2f}°",
            f"Roll:    {self.orientation.roll:7.2f}°",
            f"Pitch:   {self.orientation.pitch:7.2f}°",
            "",
            f"Calib: S{self.calibration[0]} G{self.calibration[1]} "
            f"A{self.calibration[2]} M{self.calibration[3]}"
        ]

        y = 20
        for line in lines:
            self._render_text(line, 20, y, Colors.WHITE)
            y += 25

        # Direction label
        direction = self._heading_to_direction(self.orientation.heading)
        self._render_text(f"Direction: {direction}", 20, self.height - 30, Colors.YELLOW)

        # Controls hint
        self._render_text("ESC to quit | Arrow keys to rotate view",
                         self.width - 350, self.height - 30, Colors.GRAY)

        # Restore 3D projection
        glMatrixMode(GL_PROJECTION)
        glPopMatrix()
        glMatrixMode(GL_MODELVIEW)
        glPopMatrix()

        glEnable(GL_DEPTH_TEST)
        glEnable(GL_LIGHTING)

    def _render_text(self, text: str, x: int, y: int, color: tuple):
        """Render text at screen position."""
        surface = self.font.render(text, True,
                                   tuple(int(c * 255) for c in color))
        data = pygame.image.tostring(surface, "RGBA", True)
        w, h = surface.get_size()

        glRasterPos2f(x, y + h)
        glDrawPixels(w, h, GL_RGBA, GL_UNSIGNED_BYTE, data)

    def _heading_to_direction(self, heading: float) -> str:
        """Convert heading angle to compass direction name."""
        directions = ['N', 'NNE', 'NE', 'ENE', 'E', 'ESE', 'SE', 'SSE',
                      'S', 'SSW', 'SW', 'WSW', 'W', 'WNW', 'NW', 'NNW']
        index = int((heading + 11.25) / 22.5) % 16
        return directions[index]

    def update(self, orientation: Orientation, calibration: tuple = None):
        """Update current orientation and calibration data."""
        self.orientation = orientation
        if calibration:
            self.calibration = calibration

    def render(self):
        """Render one frame."""
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)

        self.set_perspective()

        # Draw static elements
        self.draw_grid()
        self.draw_axes()
        self.draw_compass()
        self.draw_heading_indicator()

        # Draw rotated aircraft/arrow using quaternion (no gimbal lock!)
        glPushMatrix()

        # Apply rotation using quaternion-derived rotation matrix
        # This avoids gimbal lock that occurs with Euler angle rotations
        rot_matrix = self.orientation.quaternion.to_rotation_matrix()
        glMultMatrixf(rot_matrix)

        self.draw_arrow_3d()

        glPopMatrix()

        # Draw 2D overlay
        self.draw_text_overlay()

        pygame.display.flip()

    def handle_events(self) -> bool:
        """Process pygame events. Returns False if should quit."""
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return False
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    return False
        return True

    def cleanup(self):
        """Clean up pygame."""
        pygame.quit()
