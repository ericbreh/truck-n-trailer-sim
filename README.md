# Truck-and-Trailer Model Predictive Control

This repository explores the implementation of Model Predictive Control (MPC) for a truck-and-trailer system. The project covers various navigation tasks including point-to-point parking, waypoint following, and dynamic obstacle avoidance.

## Kinematic Model

The core of the simulation uses a nonlinear kinematic bicycle model extended for a trailer. The system state $q$ and control input $u$ are defined as:

$$
q = \begin{bmatrix} x \\ y \\ \theta_t \\ \theta_l \\ v \\ \phi \end{bmatrix}, \quad u = \begin{bmatrix} a \\ \dot{\phi} \end{bmatrix}
$$

Where:

- $(x, y)$: Position of the truck's rear axle (hitch point)
- $\theta_t$: Heading angle of the truck
- $\theta_l$: Heading angle of the trailer
- $v$: Longitudinal velocity of the truck
- $\phi$: Steering angle of the front wheels
- $a$: Longitudinal acceleration
- $\dot{\phi}$: Steering rate

### Continuous Dynamics

$$
\begin{aligned}
\dot{x} &= v \cos(\theta_t) \\
\dot{y} &= v \sin(\theta_t) \\
\dot{\theta}_t &= \frac{v}{L} \tan(\phi) \\
\dot{\theta}_l &= \frac{v}{d} \sin(\theta_t - \theta_l) \\
\dot{v} &= a \\
\dot{\phi} &= \dot{\phi}
\end{aligned}
$$

$L$ is the truck wheelbase, $d$ is the distance from the hitch to the trailer axle.

## MPC Formulation

The control problem is formulated as a Constrained Nonlinear Optimization problem solved at each time step over a prediction horizon $N$.

### Cost Function
The objective is to minimize the tracking error and control effort:

$$
J = \sum_{k=1}^{N} \|q_k - q_{ref}\|_Q^2 + \sum_{k=0}^{N-1} \|u_k\|_R^2
$$

### Constraints
- **System Dynamics**: Discretized using Forward Euler integration.
- **Actuator Limits**:
  - $v_{min} \le v \le v_{max}$
  - $\phi_{min} \le \phi \le \phi_{max}$
  - $a_{min} \le a \le a_{max}$
  - $\dot{\phi}_{min} \le \dot{\phi} \le \dot{\phi}_{max}$
- **Jackknife Prevention**: $|\theta_t - \theta_l| \le \gamma_{max}$ (typically $\pi/2$).
- **Collision Avoidance**: Distance constraints between the vehicle and obstacle coordinates.