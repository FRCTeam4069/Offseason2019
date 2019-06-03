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

        self.k = 0

        self.state_data = np.genfromtxt("States.csv", delimiter=",")
        self.input_data = np.genfromtxt("Voltages.csv", delimiter=",")
        self.ref_data = np.genfromtxt("References.csv", delimiter=",")

        self.data_t = self.state_data[1:, 0].T
        self.data_y = self.state_data[1:, 1:3].T
        self.data_u = self.input_data[1:, 1:2].T
        self.data_r = self.ref_data[1:, 1:3].T

        frccnt.System.__init__(
            self, np.zeros((2, 1)), np.array([[-12.0]]), np.array([[12.0]]), dt
        )

    def create_model(self, states):
        # Number of motors
        num_motors = 2.0
        # Elevator carriage mass in kg
        m = 9.072
        # Radius of pulley in meters
        r = 0.4 * 0.0231
        # Gear ratio
        G = 40.0

        mot775 = frccnt.models.MOTOR_775PRO
        mot775.Kv *= 0.5

        model = frccnt.models.elevator(mot775, num_motors, m, r, G)
        model.C = np.array([[1, 0], [0, 1]])
        model.D = np.array([[0], [0]])
        return model

    def design_controller_observer(self):
        # Desired max deviation from position and velocity respectively
        q = [0.02, 0.4]
        r = [12.0]
        self.design_lqr(q, r)
        self.design_two_state_feedforward(q, r)

        # 0.02 corresponds to some confidence in x evolution in our state.
        # 0.001 corresponds with low confidence in v evolution
        kq = [0.4, 0.5]
        kr = [1.0, 0.01]
        self.design_kalman_filter(kq, kr)

    def update_plant(self):
        self.x = self.sysd.A @ self.x + self.sysd.B @ self.u
        self.y = self.data_y[:, self.k:self.k + 1]

    def correct_observer(self):
        self.x_hat += self.kalman_gain * (self.y - self.sysd.C @ self.x_hat - self.sysd.D @ self.u)

    def update_controller(self, next_r):
        self.u = self.data_u[:, self.k:self.k + 1]
        self.r = next_r
        self.k += 1


def main():
    dt = 0.01
    elevator = Elevator(dt)
    elevator.export_cpp_coeffs("Elevator", "subsystems/")
    # elevator.export_kotlin_coeffs("Elevator")

    t = elevator.data_t

    refs = []

    for k in range(len(t)):
        r = elevator.data_r[:, k:k + 1]
        refs.append(r)

    if "--save-plots" in sys.argv or "--noninteractive" not in sys.argv:
        try:
            import slycot

            plt.figure(1)
            elevator.plot_pzmaps()
        except ImportError:  # Slycot unavailable. Can't show pzmaps.
            pass
    if "--save-plots" in sys.argv:
        plt.savefig("elevator_pzmaps.svg")

    plt.figure(1)
    x_rec, ref_rec, u_rec, y_rec = elevator.generate_time_responses(t, refs)
    subplot_max = elevator.sysd.states + elevator.sysd.inputs
    for i in range(elevator.sysd.states):
        plt.subplot(subplot_max, 1, i + 1)
        plt.ylabel(elevator.state_labels[i])
        if i == 0:
            plt.title("Time-domain responses")

        plt.plot(t, elevator.extract_row(x_rec, i), label="Estimated state")
        plt.plot(t, elevator.extract_row(ref_rec, i), label="Reference")
        plt.plot(t, elevator.extract_row(y_rec, i), label="Output")
        plt.legend()

    for i in range(elevator.sysd.inputs):
        plt.subplot(subplot_max, 1, elevator.sysd.states + i + 1)
        plt.ylabel(elevator.u_labels[i])
        plt.plot(t, elevator.extract_row(u_rec, i), label="Control effort")
        plt.legend()

    # if "--save-plots" in sys.argv or "--noninteractive" not in sys.argv:
    #     plt.figure(2)
    #     x_rec, ref_rec, u_rec = elevator.generate_time_responses(t, refs)
    #     elevator.plot_time_responses(t, x_rec, ref_rec, u_rec)
    # if "--save-plots" in sys.argv:
    #     plt.savefig("elevator_response.svg")
    if "--noninteractive" not in sys.argv:
        plt.show()


if __name__ == "__main__":
    main()
