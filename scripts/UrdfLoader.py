import xml.etree.ElementTree as ET

"""
  @author: Mehmet Kahraman
  @date: 19.11.2024
  @about: URDF Loader Python Class
"""

class Joint:
    def __init__(self):
        self.name = ""
        self.type = ""
        
        self.parent = ""
        self.child = ""
        
        self.origin_xyz = [0.0, 0.0, 0.0]
        self.origin_rpy = [0.0, 0.0, 0.0]
        self.axis_xyz = [0.0, 0.0, 0.0]
        
        self.lower_limit = 0.0
        self.limit_upper = 0.0
        self.upper_limit = 0.0
        self.effort = 0.0
        self.velocity = 0.0


class Link:
    def __init__(self):
        self.name = ""
        self.mass = 0.0
        
        self.origin_xyz = [0.0, 0.0, 0.0]
        self.origin_rpy = [0.0, 0.0, 0.0]
        
        self.mesh_path = ""
        self.mesh_color = [0.0, 0.0, 0.0]
        
        self.has_inertia = False
        self.inertia = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        # inertia --> [Ixx, Ixy, Ixz, Iyy, Izz, Iyz]


class UrdfLoader:
    
    def __init__(self):
        self.urdf_path = ""
        self.robot_name = ""
        
        self.joints = []
        self.links = []
        
        self.base_link = ""
        self.end_link = ""

        self.robot_tree = None

    def load_urdf(self, file_path) -> bool:
        """
        load urdf from file
        """
        try:
            tree = ET.parse(file_path)
            self.robot_tree = tree.getroot()
            
            if self.robot_tree.tag != "robot":
                raise ValueError("No 'robot' element found in URDF")
            
            self.urdf_path = file_path
            
            self.process_robot_params()
            self.sort_joints_and_links()
        
        except Exception as e:
            print(f"Failed to load URDF: {e}")
            return False
        
        return True

    def process_robot_params(self):
        
        self.robot_name = self.robot_tree.attrib.get("name")

        # parse joints
        for joint in self.robot_tree.findall("joint"):
            
            joint_data = Joint()
            joint_data.name = joint.attrib.get("name")
            joint_data.type = joint.attrib.get("type")
            
            if (joint_data.type != "revolute") and (joint_data.type != "fixed"):
                raise ValueError("Unsupported joint type: " + joint_data.type)
            
            limit = joint.find("limit")
            if limit is not None:
                joint_data.lower_limit = float(limit.attrib.get("lower", 0.0))
                joint_data.upper_limit = float(limit.attrib.get("upper", 0.0))
                joint_data.effort = float(limit.attrib.get("effort", 0.0))
                joint_data.velocity = float(limit.attrib.get("velocity", 0.0))
            else:
                joint_data.lower_limit = 0.0
                joint_data.upper_limit = 0.0
                joint_data.effort = 0.0
                joint_data.velocity = 0.0
            
            parent = joint.find("parent")
            if parent is not None:
                joint_data.parent = parent.attrib.get("link")
            else:
                raise ValueError("Parent link not found for joint: " + joint_data.name)
            
            child = joint.find("child")
            if child is not None:
                joint_data.child = child.attrib.get("link")
            else:
                raise ValueError("Child link not found for joint: " + joint_data.name)
            
            origin = joint.find("origin")
            if origin is not None:
                joint_data.origin_xyz = self.string_to_floats(origin.attrib.get("xyz"))
                joint_data.origin_rpy = self.string_to_floats(origin.attrib.get("rpy"))
            else:
                raise ValueError("No origin element found in joint: " + joint_data.name)
            
            axis = joint.find("axis")
            if axis is not None:
                joint_data.axis_xyz = self.string_to_floats(axis.attrib.get("xyz"))
            else:
                joint_data.axis_xyz = [0.0, 0.0, 0.0]
            
            self.joints.append(joint_data)

        # parse links
        for link in self.robot_tree.findall("link"):
            
            link_data = Link()
            link_data.name = link.attrib.get("name")
            
            inertial = link.find("inertial")
            if inertial is not None:
                link_data.has_inertia = True
                
                inertial_origin = inertial.find("origin")
                if inertial_origin is not None:
                    link_data.origin_xyz = self.string_to_floats(inertial_origin.attrib.get("xyz"))
                    link_data.origin_rpy = self.string_to_floats(inertial_origin.attrib.get("rpy"))
                else:
                    raise ValueError("No origin found in link: " + link_data.name)
                
                mass = inertial.find("mass")
                if mass is not None:
                    link_data.mass = float(mass.attrib.get("value"))
                else:
                    raise ValueError("No mass found in link: " + link_data.name)
                
                inertial_params = inertial.find("inertia")
                if inertial_params is not None:
                    link_data.inertia[0] = float(inertial_params.attrib.get("ixx"))
                    link_data.inertia[1] = float(inertial_params.attrib.get("ixy"))
                    link_data.inertia[2] = float(inertial_params.attrib.get("ixz"))
                    link_data.inertia[3] = float(inertial_params.attrib.get("iyy"))
                    link_data.inertia[4] = float(inertial_params.attrib.get("izz"))
                    link_data.inertia[5] = float(inertial_params.attrib.get("iyz"))
                else:
                    raise ValueError("No inertia parameters found in link: " + link_data.name)
            
            else:
                link_data.mass = 0.0
                link_data.origin_xyz = [0.0, 0.0, 0.0]
                link_data.origin_rpy = [0.0, 0.0, 0.0]
                link_data.inertia = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
                link_data.has_inertia = False
            
            visual = link.find("visual")
            if visual is not None:
                link_data.mesh_path = visual.find("geometry").find("mesh").attrib.get("filename")
                link_data.mesh_color = self.string_to_floats(visual.find("material").find("color").attrib.get("rgba"))[:3]
            else:
                link_data.mesh_path = ""
                link_data.mesh_color = [0.0, 0.0, 0.0]
            
            self.links.append(link_data)

    def sort_joints_and_links(self):
        sorted_joints = []
        sorted_jnt_idxes = []
        sorted_links = []
        
        # find root link and first joint
        for i in range(len(self.joints)):
            is_root = True
            for j in range(len(self.joints)):
                if i == j:
                    continue
                
                if self.joints[i].parent == self.joints[j].child:
                    is_root = False
                    break
            
            if is_root == True:
                sorted_jnt_idxes.append(i)
                sorted_joints.append(self.joints[i])
                sorted_links.append(self.get_link_by_name(self.joints[i].parent))
                break
        
        if is_root == False:
            raise ValueError("No root link found !")

        # add remaining joints and links in order
        idx = 0
        while True:
            last_added_jnt = sorted_joints[idx]
            
            skip = False
            next_jnt_found = False
            for i in range(len(self.joints)):
                for j in range(len(sorted_jnt_idxes)):
                    if i == sorted_jnt_idxes[j]:
                        skip = True
                    
                if skip == True:
                    skip = False
                    continue
                    
                if last_added_jnt.child == self.joints[i].parent:
                    sorted_jnt_idxes.append(i)
                    sorted_joints.append(self.joints[i])
                    sorted_links.append(self.get_link_by_name(self.joints[i].parent))
                    next_jnt_found = True
                    break
                
            if len(sorted_jnt_idxes) == len(self.joints):
                sorted_links.append(self.get_link_by_name(sorted_joints[len(self.joints)-1].child))
                break
            
            if next_jnt_found == False:
                raise ValueError("child " + last_added_jnt.child + " of " + last_added_jnt.name + " not found !!")

            idx = idx + 1

        self.joints = sorted_joints.copy()
        self.links = sorted_links.copy()

    def set_kinematic_chain(self, base_link, end_link):
        self.base_link = base_link
        self.end_link = end_link
        
        if base_link == end_link:
            raise ValueError("base_link and end_link cannot be the same")
        
        base_found = False
        end_found = False
        for i in range(len(self.links)):
            if self.links[i].name == base_link:
                base_found = True
                base_link_id = i
            if self.links[i].name == end_link:
                end_found = True
                end_link_id = i
            
            if base_found and end_found:
                break
        
        if (base_found == False) or (end_found == False):
            raise ValueError("base_link or end_link not found in links")
        
        if base_link_id > end_link_id:
            raise ValueError("base_link and end_link not in kinematic order !!")
        
        new_links = []
        new_joints = []
        
        for i in range(base_link_id, end_link_id+1):
            new_links.append(self.links[i])
        
        for i in range(len(self.joints)):
            if self.joints[i].parent == base_link:
                first_jnt_id = i
            if self.joints[i].child == end_link:
                end_jnt_id = i
        
        for i in range(first_jnt_id, end_jnt_id+1):
            new_joints.append(self.joints[i])
        
        self.joints = new_joints.copy()
        self.links = new_links.copy()

    def get_urdf_path(self):
        return self.urdf_path
    
    def get_total_num_joints(self):
        return len(self.joints)

    def get_num_revolute_joints(self):
        counter = 0
        for joint in self.joints:
            if joint.type == "revolute":
                counter = counter + 1
        return counter
    
    def get_all_joint_names(self):
        joint_names = []
        for joint in self.joints:
            joint_names.append(joint.name)
        return joint_names
    
    def get_revolute_joint_names(self):
        joint_names = []
        for joint in self.joints:
            if joint.type == "revolute":
                joint_names.append(joint.name)
        return joint_names

    def get_all_joints(self):
        return self.joints
    
    def get_revolute_joints(self):
        revolute_joints = []
        for joint in self.joints:
            if joint.type == "revolute":
                revolute_joints.append(joint)
        return revolute_joints

    def get_joint_by_name(self, name):
        for joint in self.joints:
            if joint.name == name:
                return joint
        raise ValueError(f"Joint with name '{name}' not found")

    def get_lower_pos_limits(self):
        lower_limits = []
        for joint in self.joints:
            if joint.type == "revolute":
                lower_limits.append(joint.lower_limit)
        return lower_limits
    
    def get_upper_pos_limits(self):
        upper_limits = []
        for joint in self.joints:
            if joint.type == "revolute":
                upper_limits.append(joint.upper_limit)
        return upper_limits

    def get_total_num_links(self):
        return len(self.links)
    
    def get_all_link_names(self):
        link_names = []
        for link in self.links:
            link_names.append(link.name)
        return link_names
    
    def get_all_links(self):
        return self.links

    def get_link_by_name(self, name):
        for link in self.links:
            if link.name == name:
                return link
        raise ValueError(f"Link with name '{name}' not found")
    
    def string_to_floats(self, string):
        float_list = [float(x) for x in string.split(" ")]
        return float_list