import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
from matplotlib.patches import Circle
from matplotlib.collections import PatchCollection


"""
Helper functions for visualizing robot drawing lines in the world frame without needing to run the robot realtime.
Only need to call plot_lines() in your main code to visualize the lines.
"""


def make_thick_segment(p1, p2, width_cm):
    """Create a rectangular polygon patch representing a thick line segment.

    Computes a rectangle of the given width centered on the segment from p1 to p2 by offsetting 
    perpendicular to the segment direction by half the width on each side.

    Parameters
    ----------
    p1 : np.ndarray of shape (2,)
        Start point of the segment (x, y) in cm.
    p2 : np.ndarray of shape (2,)
        End point of the segment (x, y) in cm.
    width_cm : float
        Width of the segment in cm.

    Returns
    -------
    Polygon
        A matplotlib Polygon patch representing the thick segment.
    None
        If p1 and p2 are the same point (zero-length segment).

    Example
    -------
    >>> poly = make_thick_segment(np.array([0, 0]), np.array([10, 0]), width_cm=0.3)
    >>> ax.add_patch(poly)
    """
    p1, p2 = np.array(p1), np.array(p2)
    direction = p2 - p1
    length = np.linalg.norm(direction)
    if length == 0:
        return None
    unit = direction / length
    perp = np.array([-unit[1], unit[0]])  # perpendicular vector
    half_w = width_cm / 2

    # 4 corners of the rectangle
    corners = [
        p1 + perp * half_w,
        p1 - perp * half_w,
        p2 - perp * half_w,
        p2 + perp * half_w,
    ]
    return Polygon(corners)


def plot_lines(lines, linewidth):
    """Plot line segments the robot will draw in the world frame. 

    Parameters
    ----------
    lines: list[list[tuple[float, float]]]
        A list contains lists of tuples. Each list is a sequence of (x, y) points in the world frame (units: cm). 
        Each line is plotted as a closed curve — the last point is automatically connected back to the first.
    linewidth : float
        Estimated brush width in cm. You should adjust this value to better visualize the actual drawing based on your brush size.

    Example
    -------
    >>> lines = [
    ...     [(10.0, 10.0), (10.5, 12.0), (11.0, 10.0)],  # triangle
    ...     [(15.0, 15.0), (15.5, 16.0), (16.0, 15.0)],  # another shape
    ... ]
    >>> plot_lines(lines, linewidth=0.3)
    """
    markersize = 0.3  # Fixed marker size in cm

    fig, ax = plt.subplots(figsize=(7, 7))

    line_patches   = []
    marker_patches = []
    for line in lines:
        pts = [np.array([p[1], p[0]]) for p in line]  # (ry, rx)
        pts.append(pts[0])  # close the line

        for i in range(len(pts) - 1):
            poly = make_thick_segment(pts[i], pts[i + 1], linewidth)
            if poly:
                line_patches.append(poly)

        for p in pts:
            marker_patches.append(Circle(p, radius=markersize))

    ax.add_collection(PatchCollection(line_patches,   facecolor='blue', edgecolor='none'))
    ax.add_collection(PatchCollection(marker_patches, facecolor='red',  edgecolor='none'))

    ax.set_xlabel('World Y (cm)')
    ax.set_ylabel('World X (cm)')
    ax.set_title('Lines and points in the world frame')
    ax.invert_yaxis()
    ax.set_aspect('equal')
    ax.autoscale()
    ax.grid(True)
    plt.tight_layout()
    plt.show()