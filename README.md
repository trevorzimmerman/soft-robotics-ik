# Soft Robotics IK (MATLAB)

This repository contains MATLAB scripts from a university course on soft robotics.  
Each script generates an animation (saved as a `.gif`) showing different inverse kinematics approaches for soft robotic arms.

## Repository Structure  
``` 
soft-robotics-ik/  
├── data/  
│   ├── Anfis1.mat  
│   ├── Anfis2.mat  
│   └── data.mat  
├── jacobian_ik_2link_arm.m  
├── anfis_ik_soft_parallel_arm.m  
├── vpasolve_ik_3link_parallel_robot.m  
└── README.md  
```

## Scripts
- **`jacobian_ik_2link_arm.m`**  
  Inverse kinematics of a 2-link soft arm using Jacobian updates.

- **`anfis_ik_soft_parallel_arm.m`**  
  Inverse kinematics of a soft parallel arm using trained ANFIS models.

- **`vpasolve_ik_3link_parallel_robot.m`**  
  Inverse kinematics of a 3-link parallel soft robot using symbolic solving (`vpasolve`).

## Usage
Run any script in MATLAB from the repository root.  
Each script creates a folder `gif` in the same directory as the script, if it doesn't already exist. The output `.gif` animations are saved in this folder.

---

The GIF results are displayed on my [portfolio website](https://trevorzimmerman.github.io/university/soft-robotics).
