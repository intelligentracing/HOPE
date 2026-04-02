# HOPE: Hitch Open Ping-Pong Embodied AI Challenge

HOPE is an open platform for humanoid robot table tennis, developed by [Hitch Interactive](https://hitchinteractive.com) (Intelligent Racing Inc.) in collaboration with the [ROAR Platform](https://roar.berkeley.edu) at UC Berkeley. The challenge invites teams to deploy whole-body humanoid controllers that can rally a ping-pong ball against human opponents or other robots, using off-the-shelf humanoid hardware and an open-source perception and planning stack.

This repository contains the **reference design documents** for the HOPE system architecture, covering motion capture setup, model-based racket planning, and reinforcement learning training for whole-body control.

## Documents

| Document | Description | Version |
|----------|-------------|---------|
| [Motion Capture System Reference Setup](HOPE_Motion_Capture_System_and_Coordinates_Reference_Setup_v0.3.md) | OptiTrack/ROS 2 arena configuration, coordinate frames, tracked object taxonomy, humanoid base_link marker setup, ball tracking, and streaming pipeline | v0.3 |
| [7DOF Racket Model-based Planner Reference Setup](HOPE_7DOF_Racket_Model_based_Planner_Reference_Setup.md) | Ball state estimation, trajectory prediction, and racket target planning (Stages 1–3 of the HITTER framework), reimplemented in the HOPE canonical frame | v0.1 |
| [WBC Simulation Training Reference Setup](HOPE_WBC_Simulation_Training_Reference_Setup.md) | SMPL-X motion acquisition, GMR retargeting, BeyondMimic RL training pipeline for whole-body control (Stage 4), with dual-backend support for Isaac Lab and mjlab | v0.5 |
| [Hardware Deployment Reference Setup](HOPE_Hardware_Deployment_Reference_Setup.md) | Real-robot deployment via `legged_control2` (G1) or AimRT (A3): ONNX inference, ROS 2 node graph, PD gain tuning, safety procedures, and competition workflow | v0.1 |

Each document contains a **Section 0 prologue** listing all implementation differences from the original HITTER paper (Su et al., arXiv:2508.21043v2).

## System Architecture

```
                    ┌─────────────────────────────┐
                    │     OptiTrack Cameras        │
                    │     (9×, 360 Hz)             │
                    └──────────┬──────────────────┘
                               │ NatNet
                               ▼
                    ┌─────────────────────────────┐
                    │  motion_capture_tracking     │
                    │  (ROS 2 Jazzy)               │
                    │                              │
                    │  Publishes:                  │
                    │   • Ball position (3D)       │
                    │   • P1/P2 base_link pose     │
                    │   • Table origin frame       │
                    └───┬──────────────┬──────────┘
                        │              │
                        ▼              ▼
              ┌──────────────┐  ┌──────────────────┐
              │  HOPE Planner │  │  Whole-Body       │
              │  (Stages 1-3) │  │  Controller       │
              │               │  │  (Stage 4)        │
              │  Ball state   │  │                    │
              │  estimation   │  │  BeyondMimic RL    │
              │  → trajectory │  │  policy (50 Hz)    │
              │  prediction   │──│                    │
              │  → racket     │  │  Receives:         │
              │  target       │  │   • Racket target  │
              │  planning     │  │   • base_link pose │
              └──────────────┘  │   • Joint encoders  │
                                │                    │
                                │  Outputs:          │
                                │   • 29-DOF joint   │
                                │     position cmds  │
                                └────────┬───────────┘
                                         │
                                         ▼
                                ┌──────────────────┐
                                │  Humanoid Robot   │
                                │  (Unitree G1 /    │
                                │   Agibot A3)      │
                                │                   │
                                │  PD controller    │
                                │  → joint torques  │
                                └──────────────────┘
```

## Key Design Decisions

**Racket tracking is prohibited.** The motion capture system tracks exactly three categories of objects: the ping-pong table origin frame (PPT), each humanoid's `base_link` (P1, P2), and the ball. No reflective markers may be placed on the racket, the robot's hand, or the wrist link. Each robot must infer its paddle's 6-DOF pose through forward kinematics from its own `base_link` + joint encoders. This is a deliberate competition constraint that tests autonomous paddle control through the robot's internal body model.

**Multi-platform support.** The reference design supports both the Unitree G1 (via Isaac Lab + PhysX with USD assets) and the Agibot Expedition A3 (via mjlab + MuJoCo Warp with MJCF assets). Both backends share the same BeyondMimic MDP formulation and export to ONNX for deployment.

**Open-source training stack.** The WBC training pipeline is built entirely on open-source code: [BeyondMimic](https://github.com/HybridRobotics/whole_body_tracking) (MIT license) for motion tracking RL, [GMR](https://github.com/YanjieZe/GMR) (MIT license) for SMPL-X to robot retargeting, and [GVHMR](https://github.com/zju3dv/GVHMR) for monocular video-to-SMPL-X extraction. The HITTER paper's trained weights are not released; all training starts from scratch.

## Supported Robots

| Robot | DOF | Simulation Backend | Model Format | Status |
|-------|-----|-------------------|--------------|--------|
| Unitree G1 | 29 | Isaac Lab + PhysX | USD | Reference platform |
| Unitree G1 EDU | 29 + hands | Isaac Lab + PhysX | USD | Supported (hand DOFs unused) |
| Agibot Expedition A3 | TBD | mjlab + MuJoCo Warp | MJCF | In development |

## Coordinate Frame Convention

All three documents share a common world frame (ROS 2 REP 103):

- **Origin**: Near-side left corner of the table surface
- **X**: Toward opponent (along the 2.74 m table length)
- **Y**: Left (along the 1.525 m table width)
- **Z**: Up
- **Table surface height**: 0.76 m above floor

The OptiTrack system must be configured with **Up Axis → Z** in Motive to match this convention.

## Prerequisites

The reference documents assume familiarity with:

- ROS 2 Jazzy
- Python 3.10+
- NVIDIA Isaac Lab 2.1.0 or mjlab
- OptiTrack Motive (or compatible motion capture system)
- PyTorch and Weights & Biases (WandB)

## Related Repositories

| Repository | Purpose |
|-----------|---------|
| [HybridRobotics/whole_body_tracking](https://github.com/HybridRobotics/whole_body_tracking) | BeyondMimic training code (Isaac Lab) |
| [mujocolab/mjlab](https://github.com/mujocolab/mjlab) | BeyondMimic training code (MuJoCo Warp) |
| [HybridRobotics/motion_tracking_controller](https://github.com/HybridRobotics/motion_tracking_controller) | ROS 2 deployment (ONNX inference) |
| [qiayuanl/legged_control2](https://qiayuanl.github.io/legged_control2_doc/) | Low-level controller framework for legged robots |
| [qiayuanl/unitree_bringup](https://github.com/qiayuanl/unitree_bringup) | Unitree robot bringup utilities |
| [unitreerobotics/unitree_rl_mjlab](https://github.com/unitreerobotics/unitree_rl_mjlab) | Unitree official mjlab integration |
| [YanjieZe/GMR](https://github.com/YanjieZe/GMR) | General Motion Retargeting (SMPL-X → robot) |
| [zju3dv/GVHMR](https://github.com/zju3dv/GVHMR) | Video-to-SMPL-X pose estimation |
| [IMRCLab/motion_capture_tracking](https://github.com/IMRCLab/motion_capture_tracking) | ROS 2 motion capture bridge |
| [google-deepmind/mujoco_warp](https://github.com/google-deepmind/mujoco_warp) | GPU-accelerated MuJoCo |
| [AimRT/aimrt](https://github.com/AimRT/aimrt) | Agibot's lightweight robotics middleware |

## References

- Su, Z., Zhang, B., Rahmanian, N., Gao, Y., Liao, Q., Regan, C., Sreenath, K., & Sastry, S. S. (2025). HITTER: A HumanoId Table TEnnis Robot via Hierarchical Planning and Learning. *arXiv:2508.21043v2*. [Project page](https://humanoid-table-tennis.github.io/)
- Liao, Q., et al. (2025). BeyondMimic: From Motion Tracking to Versatile Humanoid Control via Guided Diffusion. *arXiv:2508.08241v4*. [Project page](https://beyondmimic.github.io/)
- Araújo, J. P., Ze, Y., Xu, P., Wu, J., & Liu, C. K. (2025). Retargeting Matters: General Motion Retargeting for Humanoid Motion Tracking. *arXiv:2510.02252*.
- Ze, Y., et al. (2025). LATENT: Learning Athletic Humanoid Tennis Skills from Imperfect Human Motion Data. *arXiv:2603.12686*.
- mjlab: A Lightweight Framework for GPU-Accelerated Robot Learning. *arXiv:2601.22074*.
- Peng, X. B., et al. (2024). SMPLOlympics: Sports Environments for Physically Simulated Humanoids. *arXiv:2407.00187*.

## License

This project is licensed under the [Apache License, Version 2.0](LICENSE). See [LICENSE](LICENSE) for the full terms.

The reference documents in this repository describe system architectures and training pipelines that depend on third-party open-source software. Each dependency carries its own license (MIT or Apache 2.0). See the Related Repositories table above for links to each project and its respective license.

## Contact

**Allen Yang**, Co-founder and CTO, Hitch Interactive (Intelligent Racing Inc.); Chair, AI Racing ROAR Platform, UC Berkeley; Founding Executive Director, VIVE AR Center, UC Berkeley

### HITTER Authors (UC Berkeley)

The HOPE reference design is adapted from the HITTER framework. The original HITTER authors are:

Zhi Su, Boren Zhang, Navid Rahmanian, Yuchen Gao, Qiayuan Liao, Colin Regan, Koushil Sreenath, S. Shankar Sastry

Hybrid Robotics Group and ROAR Platform, University of California, Berkeley
