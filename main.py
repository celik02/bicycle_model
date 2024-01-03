import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import odeint
import math
import time
# Bicycle model differential equations
def bicycle_model(state, t, delta):
    """
    :param state: State vector [x, y, theta, v]
    :param t: Time [s]
    :param delta: Steering angle [rad]
    :return: Derivatives [dx/dt, dy/dt, dtheta/dt, dv/dt]
    """
    L = 0.2  # Wheelbase [m]
    x, y, theta, v = state
    dxdt = v * np.cos(theta)
    dydt = v * np.sin(theta)
    dthetadt = v / L * np.tan(delta)
    dvdt = 0.2 if t<=5 else -0.2  # Assuming constant velocity for simplicity
    return [dxdt, dydt, dthetadt, dvdt]

# Simulation parameters
initial_state = [0, 0, 0, 0.0]  # Initial state [x, y, theta, v]
steering_angle = np.deg2rad(25)  # Steering angle (radians)
L = 0.2 # Wheelbase [m]
min_turning_radius = 0.4 #  minimum turning radius 0.4 [m]
max_steering_angle = math.atan(L/0.4) # max steering angle [rad]
if steering_angle >=max_steering_angle:
    raise Exception("Steering angle is higher than max steering angle")
simulation_time = np.linspace(0, 10, 100)  # Time vector

# Solve the differential equations
solution = odeint(bicycle_model, initial_state, simulation_time, args=(steering_angle,))

## pass velocity and steering angle to LIMO to observe how it behaves. 
from pylimo import limo
limo=limo.LIMO()
limo.EnableCommand()
vel_limo = np.zeros(100)
steering_angle_limo = np.zeros(100)
wheel_odom = np.zeros(100)
for i,v in enumerate(solution[:,3]):
    limo.SetMotionCommand(linear_vel=v, steering_angle=steering_angle)
    vel_limo[i] = limo.GetLinearVelocity()
    steering_angle_limo[i] = limo.GetSteeringAngle()
    wheel_odom[i] = limo.GetRightWheelOdom()
    ##TODO get position from optitrack 
    time.sleep(0.1)  ## determined by spacing in simulation time.


# Plot the results
plt.figure(figsize=(12, 6))

# Plot 2D position
plt.subplot(2, 2, 1)
plt.plot(solution[:, 0], solution[:, 1])
plt.title('2D Position')
plt.xlabel('X Position (m)')
plt.ylabel('Y Position (m)')

# Plot other variables
plt.subplot(2, 2, 2)
plt.plot(simulation_time, solution[:, 2])
plt.title('Theta (Orientation)')
plt.xlabel('Time (s)')
plt.ylabel('Theta (rad)')

plt.subplot(2, 2, 3)
plt.plot(simulation_time, solution[:, 3])
plt.plot(simulation_time, vel_limo)
plt.legend(['Simulation', 'Limo'])
plt.title('Velocity')
plt.xlabel('Time (s)')
plt.ylabel('Velocity (m/s)')

vel_error =  np.linalg.norm(vel_limo-solution[:,3])
print(vel_error)
print(wheel_odom)
plt.tight_layout()
plt.show()
