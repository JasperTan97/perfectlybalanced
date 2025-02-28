from dm_control import mjcf
import mujoco

# Load URDF file and convert to MJCF
mjcf_model = mjcf.from_urdf_model("description/perfbalanced.urdf")

# Save the MJCF file
with open("converted_model.xml", "w") as f:
    f.write(mjcf_model.to_xml_string())
    
print("Conversion complete! MJCF saved as converted_model.xml")
