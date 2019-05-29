#!/usr/bin/env python3

# Avoid needing display if plots aren't being shown
import sys

if "--noninteractive" in sys.argv:
    import matplotlib as mpl

    mpl.use("svg")

import frccontrol as frccnt
import matplotlib.pyplot as plt
import numpy as np


class Elevator(frccnt.System):
    def __init__(self, dt):
        """Elevator subsystem.

        Keyword arguments:
        dt -- time between model/controller updates
        """
        state_labels = [("Position", "m"), ("Velocity", "m/s")]
        u_labels = [("Voltage", "V")]
        self.set_plot_labels(state_labels, u_labels)

        frccnt.System.__init__(
            self, np.zeros((2, 1)), np.array([[-12.0]]), np.array([[12.0]]), dt
        )

    def create_model(self, states):
        # Number of motors
        num_motors = 2.0
        # Elevator carriage mass in kg
        m = 9.072
        # Radius of pulley in meters
        r = 0.0231
        # Gear ratio
        G = 40.0

        return frccnt.models.elevator(frccnt.models.MOTOR_775PRO, num_motors, m, r, G)

    def design_controller_observer(self):
        # Desired max deviation from position and velocity respectively
        q = [0.02, 0.4]
        r = [12.0]
        self.design_lqr(q, r)
        self.design_two_state_feedforward(q, r)

        # 0.02 corresponds to some confidence in x evolution in our state.
        # 0.001 corresponds with low confidence in v evolution
        kq = [0.02, 0.001]
        kr = [0.0001]
        self.design_kalman_filter(kq, kr)


def main():
    dt = 0.01
    elevator = Elevator(dt)
    elevator.export_cpp_coeffs("Elevator", "subsystems/")
    # elevator.export_kotlin_coeffs("Elevator")

    if "--save-plots" in sys.argv or "--noninteractive" not in sys.argv:
        try:
            import slycot

            plt.figure(1)
            elevator.plot_pzmaps()
        except ImportError:  # Slycot unavailable. Can't show pzmaps.
            pass
    if "--save-plots" in sys.argv:
        plt.savefig("elevator_pzmaps.svg")

    # Set up graphing
    l0 = 0.1
    l1 = l0 + 2.5
    l2 = l1 + 2.5
    l3 = l2 + 2.5
    l4 = l3 + 2.5
    l5 = l4+ 0.1
    t = np.arange(0, l5 + 5.0, dt)

    refs = []

    # Generate references for simulation
    for i in range(len(t)):
        if t[i] < l0:
            r = np.array([[0.0], [0.0]])
        elif t[i] < l1:
            r = np.array([[0.0711], [0.0]])
        elif t[i] < l2:
            r = np.array([[0.3988], [0.0]])
        elif t[i] < l3:
            r = np.array([[0.7595], [0.0]])
        elif t[i] < l4:
            r = np.array([[0.0813], [0.0]])
        else:
            r = np.array([[0.0], [0.0]])
        refs.append(r)

    if "--save-plots" in sys.argv or "--noninteractive" not in sys.argv:
        plt.figure(2)
        x_rec, ref_rec, u_rec = elevator.generate_time_responses(t, refs)
        elevator.plot_time_responses(t, x_rec, ref_rec, u_rec)
    if "--save-plots" in sys.argv:
        plt.savefig("elevator_response.svg")
    if "--noninteractive" not in sys.argv:
        plt.show()


if __name__ == "__main__":
    main()
