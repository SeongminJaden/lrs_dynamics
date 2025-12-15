flowchart TD
    A[Simulation Start] --> B[Initial State PV, Attitude]
    B --> C[Orbit Propagator 6-DoF Loop]

    C -->|Request r(t)| D[Gazebo Gravity Plugin<br/>(GGM05C)]
    D -->|a_gravity| C

    C -->|Publish PV| E[Perturbation Nodes]
    E -->|a_drag / a_srp / a_3body| F[Perturbation Aggregator]
    F -->|a_pert| C

    G[ΔV / Thrust Manager] -->|ΔV Event| C
    H[Attitude & Arm Dynamics] -->|I(t), r_cm(t), τ| C

    C --> I[Numerical Integration Δt]
    I --> J[Updated PV + Attitude]

    J --> K[TF Broadcaster]
    J --> L[Gazebo Model Update]
    J --> M[RViz / Logging]
