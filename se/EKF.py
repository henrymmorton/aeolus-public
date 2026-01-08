import math
from mpmath import sec
import random
import numpy as np

class BicycleEKF:

    def __init__(self, control_covar, meas_covar, vehicle):

        # Vehicle variables
        self.a2a_length = vehicle.a2a_length
        
        # State variables
        self.control_covar = control_covar
        self.meas_covar = meas_covar

        # Useful constants
        self.N = control_covar.shape[0]

        
    def propogate_state(self, prev_state, u, dt):

        # Decant previous state
        x_prev = prev_state[0]
        y_prev = prev_state[1]
        theta_prev = prev_state[2]

        # Decant inputs
        v = u[0]
        delta = u[1]

        # Propogate state according to motion model
        w = (v * math.tan(delta)) / self.a2a_length
        theta = theta_prev + w * dt
        vx = v * math.cos(theta)
        vy = v * math.sin(theta)
        x = x_prev + vx * dt
        y = y_prev + vy * dt

        predicted_state = np.array([x, y, theta, vx, vy, w])

        return predicted_state

    def calc_state_jac(self, theta_prev, dt):

        state_jac = np.array([1, 0, 0, dt, 0, 0],
                             [0, 1, 0, 0, dt, 0],
                             [0, 0, 1, 0, 0, dt],
                             [0, 0, -math.sin(theta_prev), 0, 0, 0],
                             [0, 0, math.cos(theta_prev), 0, 0, 0],
                             [0, 0, 0, 0, 0, 0])

        return state_jac

    def calc_control_jac(self, u):

        theta_prev = self.EKF_state[2]
        v = u[0]
        delta = u[1]

        input_jac = np.array([0, 0],
                             [0, 0],
                             [0, 0],
                             [math.cos(theta_prev), 0],
                             [math.sin(theta_prev), 0],
                             [(math.tan(delta) / self.a2a_length), (v / self.a2a_length) * (sec(delta))**2])

        return input_jac

    def prediction_step(self, prev_state, prev_state_covar, u, dt):

        predicted_state = self.propogate_state(prev_state, u, dt)
        state_jac = self.calc_state_jac
        control_jac = self.calc_control_jac(u)
        
        state_term = np.matmul(state_jac, np.matmul(prev_state_covar, np.transpose(state_jac)))
        control_term = np.matmul(control_jac, np.matmul(self.control_covar, np.transpose(control_jac)))
        predicted_state_covar = state_term + control_term

        return predicted_state, predicted_state_covar

    def calc_meas_pred(self, x_bar):

        meas_prediction = np.array(x_bar[0], x_bar[1], x_bar[2], x_bar[5])

        return meas_prediction

    def calc_meas_jac(self, x_bar):

        meas_jac = np.array(
                   [[1, 0, 0, 0, 0, 0],
                    [0, 1, 0, 0, 0, 0],
                    [0, 0, 1, 0, 0, 0],
                    [0, 0, 0, 1, 0, 0]])

        return meas_jac

    def calc_kalman_gain(self, x_bar_covar, meas_jac, meas_covar):

        meas_jac_T = np.transpose(meas_jac)
        inv_term = np.linalg.inv(np.matmul(meas_jac, np.matmul(x_bar_covar, meas_jac_T)) + meas_covar)
        kalman_gain = np.matmul(x_bar_covar, np.matmul(meas_jac_T, inv_term))

        return kalman_gain

    def correction_step(self, x_bar, x_bar_covar, z):

        meas_jac = self.calc_meas_jac(x_bar)
        K = self.calc_kalman_gain(x_bar_covar, meas_jac, self.meas_covar)
        z_bar = self.calc_meas_pred(x_bar)
        state_estimate = x_bar + np.matmul(K, (z - z_bar))
        covar_estimate = np.matmul((np.identitiy(self.N) - np.matmul(K, meas_jac)), x_bar_covar)

        return state_estimate, covar_estimate

    def EKF_state_update(self, prev_state, prev_covar, u, z, dt):

        predicted_state, predicted_covar = self.prediction_step(prev_state, prev_covar, u, dt)
        state_estimate, covar_estimate = self.correction_step(predicted_state, predicted_covar, z)

        self.EKF_state = state_estimate
        self.state_covar = covar_estimate
        
