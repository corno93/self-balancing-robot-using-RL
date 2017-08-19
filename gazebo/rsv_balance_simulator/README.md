# rsv_balance_simulator
Simulator packages for Robosavvy's balancing platform

---
## Packages 

* `rsv_balance_gazebo` 

  Package containing all necessary tools to run self-balancing platform on simulation:
  * Gazebo models.
  * Gazebo worlds.
      1. Empty.
      1. Inclined terrain.
      1. Natural terrain.
  * Launch files for:
      1. Gazebo worlds.
      1. RVIZ simulation visualization.
  * Gazebo plugin for the platform model which behaves just like the real thing.

* `rsv_balance_gazebo_control`
  
  Self-balancing control algorithm for the Gazebo plugin.
  
---
### ToDo: Further improvements

1. Refactor controller for better generalization. So we can easily test other control methods.
2. URDF parser for automatic building of control matrices and control gains.
