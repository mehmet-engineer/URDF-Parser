from UrdfLoader import UrdfLoader

"""
  @author: Mehmet Kahraman
  @date: 19.11.2024
  @about: URDF Loader Python Class
"""

print("Urdf Loader Example --- \n")

urdf = "urdf/elfin3.urdf"
robot_model = UrdfLoader()

success = robot_model.load_urdf(urdf)

if success == False:
    print("Failed to load URDF")
    quit()
    
# print robot info
print(" // ROBOT INFO //////// ")
print("Robot Name:", robot_model.robot_name)
print("Urdf path:", robot_model.urdf_path)

# print kinematic info
print("\n // KINEMATIC INFO //////// ")
base_link = "elfin_base"
end_link = "elfin_link6"
robot_model.set_kinematic_chain(base_link, end_link)
print("base_link:", robot_model.base_link)
print("end_link:", robot_model.end_link)
print("total_num_joints:", robot_model.get_total_num_joints())
print("num_revolute_joints:", robot_model.get_num_revolute_joints())
print("total_num_links:", robot_model.get_total_num_links())

# print joint and link names
print("\n // JOINTS AND LINKS //////// ")
print("joint_names:", robot_model.get_all_joint_names())
print("link_names:", robot_model.get_all_link_names())

# print lower and upper position limits
print("\n // POSITION LIMITS //////// ")
print("lower_pos_limits:", robot_model.get_lower_pos_limits())
print("upper_pos_limits:", robot_model.get_upper_pos_limits())

# access joints and links by name
print("\n // ACCESS JOINTS AND LINKS //////// ")
print("velocity limit of elfin_joint1:", robot_model.get_joint_by_name("elfin_joint1").velocity)
print("mesh color of elfin_link4:", robot_model.get_link_by_name("elfin_link4").mesh_color)

print()