# URDF-Parser
URDF Parser and loader Python Class without any extra dependency.

**Author: Mehmet KAHRAMAN | Date: 19.11.2024 | Last Update: 25.11.2024**

Assumptions
---
- Joint axis elements must be one of -1, 0, 1 numbers.
- Transforms are calculated from joint xyz and rpy. Visual origin of link are neglected. 

Requirements
---
- Python3
- Python xml

Run Example Script
---
```
python3 scripts/urdf_load_example.py
```

Example API Usage
---
```
robot_model = UrdfLoader()
success = robot_model.load_urdf("urdf/elfin3.urdf")

print("Robot Name:", robot_model.robot_name)
print("Urdf path:", robot_model.urdf_path)

base_link = "elfin_base"
end_link = "elfin_link6"
robot_model.set_kinematic_chain(base_link, end_link)

print("total_num_joints:", robot_model.get_total_num_joints())
print("num_revolute_joints:", robot_model.get_num_revolute_joints())
print("total_num_links:", robot_model.get_total_num_links())

print("joint_names:", robot_model.get_all_joint_names())
print("link_names:", robot_model.get_all_link_names())

print("lower_pos_limits:", robot_model.get_lower_pos_limits())
print("upper_pos_limits:", robot_model.get_upper_pos_limits())

print(robot_model.get_joint_by_name("elfin_joint1").velocity)
print(robot_model.get_link_by_name("elfin_link4").mesh_color)
```
