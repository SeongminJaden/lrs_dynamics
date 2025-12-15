# End-to-End Flow

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


# ROS Node Graph

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



# Integration sequenceDiagram

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

