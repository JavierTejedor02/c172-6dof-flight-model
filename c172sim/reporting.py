"""Dependency-light trajectory export shared by the demonstration scripts."""
import numpy as np
from .dynamics import STATE_NAMES


def save_csv(filename, simulation):
    """Write time, all twelve states, altitude and the four actual inputs."""
    values = np.column_stack((simulation.time, simulation.states,
                              -simulation.states[:, 11], simulation.controls))
    header = ("time_s," + ",".join(STATE_NAMES)
              + ",altitude_m,elevator_rad,aileron_rad,rudder_rad,thrust_N")
    np.savetxt(filename, values, delimiter=",", header=header, comments="", fmt="%.12g")
