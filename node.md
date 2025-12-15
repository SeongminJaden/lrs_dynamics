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



graph LR
    GZ[Gazebo<br/>GGM05C Plugin]
    OP[Orbit Propagator<br/>6-DoF HPOP]
    DRAG[Drag Node]
    SRP[SRP Node]
    TB[3rd Body Node]
    PA[Perturbation Aggregator]
    DV[ΔV / Thrust Manager]
    ATT[Attitude / Arm Dynamics]
    TF[TF Broadcaster]
    RV[RViz / Logger]

    GZ -->|a_gravity| OP

    OP -->|PV| DRAG
    OP -->|PV| SRP
    OP -->|PV| TB

    DRAG --> PA
    SRP --> PA
    TB --> PA

    PA -->|a_pert| OP
    DV -->|ΔV Event| OP
    ATT -->|I(t), r_cm, τ| OP

    OP --> TF
    OP --> RV
