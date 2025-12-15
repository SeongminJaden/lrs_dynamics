## ROS2-Based LEO Satellite & Robotic Arm Integrated Simulation

### 1. Executive Summary

This project builds a high-fidelity simulation platform integrating ROS2 and Gazebo for LEO satellite dynamics and a satellite-mounted robotic arm. It includes real-time orbit propagation using HPOP (or optional SGP4), satellite attitude and manipulator dynamics, coupled dynamics, and environmental physics including position-dependent gravity using GGM05C, atmospheric drag, and solar radiation.

### 2. Background & Motivation

* Robotic arms are essential for space services (repair, assembly, cargo handling).
* Ground verification requires combined orbit propagation and manipulator dynamics.
* Gazebo alone cannot simulate high-precision orbital motion or LEO gravity accurately.

### 3. Objectives

1. HPOP-based real-time orbit propagator ROS2 node
2. Orbit-to-Gazebo bridge for model updates
3. Satellite + 6-DoF manipulator URDF/SDF modeling
4. Coupled dynamics simulation (arm torque → satellite attitude)
5. Verification scenarios: approach, grasp, manipulation

### 4. Scope & Assumptions

* ROS2 Humble/Iron
* Gazebo Fortress/Garden
* HPOP numerical integration with dt-based publishing
* Optional SGP4 for rapid prototyping
* Development PC: quad-core+, 16GB RAM
* dt: 0.1–1.0 s recommended

### 5. System Architecture
#### 5.1 End-to-End Flow
```mermaid
flowchart TD
    A[Simulation Start] --> B[Initial State PV and Attitude]
    B --> C[Orbit Propagator 6DoF]

    C -->|Request position| D[Gazebo Gravity Plugin GGM05C]
    D -->|Gravity acceleration| C

    C -->|Publish PV| E[Perturbation Nodes]
    E -->|Drag SRP ThirdBody| F[Perturbation Aggregator]
    F -->|Total perturbation| C

    G[DeltaV Thrust Manager] -->|DeltaV Event| C
    H[Attitude Arm Dynamics] -->|Inertia COM Torque| C

    C --> I[Numerical Integration Dt]
    I --> J[Updated PV and Attitude]

    J --> K[TF Broadcaster]
    J --> L[Gazebo Model Update]
    J --> M[RViz Logging]
```

#### 5.2 ROS Node Graph
```mermaid
graph LR
    GZ[Gazebo GGM05C Plugin]
    OP[Orbit Propagator 6DoF HPOP]
    DRAG[Drag Node]
    SRP[SRP Node]
    TB[Third Body Node]
    PA[Perturbation Aggregator]
    DV[DeltaV Thrust Manager]
    ATT[Attitude Arm Dynamics]
    TF[TF Broadcaster]
    RV[RViz Logger]

    GZ -->|GravityAccel| OP

    OP -->|PV| DRAG
    OP -->|PV| SRP
    OP -->|PV| TB

    DRAG --> PA
    SRP --> PA
    TB --> PA

    PA -->|TotalPerturb| OP
    DV -->|DeltaVEvent| OP
    ATT -->|InertiaAndTorque| OP

    OP --> TF
    OP --> RV
```


#### 5.3 Integration sequenceDiagram
```mermaid
sequenceDiagram
    participant ROS
    participant OP as Orbit Propagator
    participant GZ as Gravity Plugin
    participant PA as Perturbation Agg.
    participant DV as ΔV Manager

    ROS->>OP: PV Update (buffered)
    DV->>OP: ΔV Event (queued)

    OP->>GZ: r(t)
    GZ-->>OP: a_gravity

    OP->>PA: r(t), v(t)
    PA-->>OP: a_pert

    OP->>OP: Apply ΔV (if t_k)
    OP->>OP: Integrate Δt
    OP->>ROS: Publish State + TF
```


**Components:**

1. Orbit Propagator Node (HPOP) → publishes `/satellite/orbit_state`
2. Orbit-to-Gazebo Bridge → ECI → Gazebo conversion
3. Gazebo World & Plugins → ModelPlugin, WorldPlugin
4. Satellite & Manipulator → URDF/SDF with 6-DoF arm
5. Control Stack → Arm + Satellite attitude controller
6. Visualization & Logging → RViz, Gazebo GUI, CSV/rosbag

**Data Flow:**

```
Initial state → HPOP → (r,v) → ECI→Gazebo → Plugin → Satellite Pose → Physics → Arm Controller → Joint Torques → Reaction → Next Step
```

### 6. Messages & Interfaces

* `/satellite/orbit_state` : `nav_msgs/Odometry`
* `/satellite/eci_state` : custom `{ header, r[3], v[3], epoch }`
* `/gazebo/set_model_state` : `gazebo_msgs/ModelState`
* Services: `/orbit/reset`, `/orbit/set_parameters`

### 7. Physics & Gravity Modeling

**Earth Gravity Potential (GOCO2025s):**

$$
V(r, \phi, \lambda) = \frac{GM}{r} \left[1 + \sum_{n=2}^{n_{max}} \left(\frac{a}{r}\right)^n \sum_{m=0}^{n} \bar{P}*{n,m}(\sin \phi)(\bar{C}*{n,m}\cos(m\lambda) + \bar{S}_{n,m}\sin(m\lambda))\right]
$$


**Fully-Normalized Associated Legendre Functions:**

$$
\bar{P}*{n,m}(x) = N*{n,m} P_{n,m}(x),\quad N_{n,m} = \sqrt{(2-\delta_{m0})(2n+1)\frac{(n-m)!}{(n+m)!}}
$$


**Gravitational Acceleration:**

$$
\mathbf{a} = -\nabla V
$$

Components:

$$
a_r = -\frac{\partial V}{\partial r},\quad a_\phi = -\frac{1}{r} \frac{\partial V}{\partial \phi},\quad a_\lambda = -\frac{1}{r\cos\phi} \frac{\partial V}{\partial \lambda}
$$


**Spherical → Cartesian:**

$$
\begin{aligned}
a_x &= a_r \cos\phi \cos\lambda - a_\phi \sin\phi \cos\lambda - a_\lambda \sin\lambda \
a_y &= a_r \cos\phi \sin\lambda - a_\phi \sin\phi \sin\lambda + a_\lambda \cos\lambda \
a_z &= a_r \sin\phi + a_\phi \cos\phi
\end{aligned}
$$


**Force in Gazebo:**

$$
\mathbf{F} = m \mathbf{a}
$$


**Coupled Satellite-Arm Dynamics:** Arm motion induces torque on satellite. Use HPOP for CoM orbit, Gazebo for attitude + arm, ModelPlugin for reaction.

### 8. Synchronization

* HPOP publish: 10 Hz (dt=0.1s)
* Gazebo update: faster than HPOP
* Frames: ECI → Gazebo world (or ENU)


### 9. Development Environment

* Ubuntu 22.04
* ROS2 Humble/Iron
* Gazebo Fortress/Garden
* MoveIt2
* C++ (plugins), Python (nodes)

### 10. Build & Run

```bash
colcon build --packages-select satellite_description satellite_plugins satellite_control satellite_moveit satellite_utils gazebo_leo_gravity
source install/setup.bash
ros2 launch satellite_sim satellite_sim.launch.py
```
