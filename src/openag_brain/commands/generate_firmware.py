import os
import subprocess

from openag_brain.models import FirmwareModuleTypeModel, FirmwareModuleModel
from openag_brain.db_names import DbName

def read_module_data(module_db, module_type_db):
    modules = {
        module_id: FirmwareModuleModel.load(module_db, module_id) for module_id
        in module_db if not module_id.startswith('_')
    }
    module_types = {
        module.type: FirmwareModuleTypeModel.load(module_type_db, module.type)
        for module in modules.values()
    }
    return modules, module_types

def download_lib(path, url):
    if os.path.isdir(path):
        subprocess.call(["git", "pull"], cwd=path)
    else:
        subprocess.call(["git", "clone", url, path])

def download_libs(module_types, lib_folder):
    for module_type_id, module_type in module_types.items():
        type_folder = os.path.join(lib_folder, module_type_id)
        download_lib(type_folder, module_type.url)
    peripheral_folder = os.path.join(
        lib_folder, "openag_peripheral"
    )
    download_lib(
        peripheral_folder,
        "http://github.com/OpenAgInitiative/openag_peripheral"
    )

def write_code(modules, module_types, f):
    # Include the ros library
    f.write("#include <ros.h>\n\n")

    # Include the headers for the module types
    for module_type in module_types.values():
        f.write("#include <{}>\n".format(module_type.header_file))
    msg_types = set()
    f.write("\n")

    # Include the required message types
    for module_type in module_types.values():
        for x in module_type.inputs.values():
            msg_types.add(x)
        for x in module_type.outputs.values():
            msg_types.add(x)
    for msg_type in msg_types:
        f.write("#include <{}.h>\n".format(msg_type))
    f.write("\n")

    # Define all of the modules
    for module in modules.values():
        module_type = module_types[module.type]
        parameters_name = module.id + "_parameters"
        f.write("String {}[] = {{{}}};\n".format(
            parameters_name, ", ".join(module.parameters)
        ))
        f.write('{class_name} {id}("{id}", {parameters});\n\n'.format(
            class_name=module_type.class_name, id=module.id,
            parameters=parameters_name
        ))

    # Define the ROS node handle
    f.write("ros::NodeHandle nh;\n\n")

    # TODO: Define subscribers from module inputs

    # Define publishers from module outputs
    publishers = []
    for module in modules.values():
        module_type = module_types[module.type]
        for output, output_type in module_type.outputs.items():
            output_id = module.id + "_" + output
            msg_class = "::".join(output_type.split("/"))
            msg_name = output_id + "_msg"
            f.write("{} {};\n".format(msg_class, msg_name))
            pub_name = "pub_" + output_id
            topic_name = "/sensors/" + output_id
            f.write('ros::Publisher {}("{}", &{});\n\n'.format(
                pub_name, topic_name, msg_name
            ))
            publishers.append(pub_name)


    # Write the setup function
    f.write("void setup() {\n")
    f.write("  Serial.begin(57600);\n")
    f.write("  nh.initNode();\n\n")
    # TODO: Register subscribers
    # Register publishers
    for publisher in publishers:
        f.write("  nh.advertise({});\n".format(publisher))
    f.write("\n")
    # Initialize the modules
    for module_id in modules.keys():
        f.write("  {}.begin();\n".format(module_id))
    f.write("}\n\n")

    # Write the loop function
    f.write("void loop() {\n")
    f.write("  nh.spinOnce();\n")
    for module in modules.values():
        module_type = module_types[module.type]
        for output in module_type.outputs.keys():
            f.write('\n  {}.get("{}");\n'.format(module.id, output))
            output_id = module.id + "_" + output
            msg_name = output_id + "_msg"
            f.write("  {}.data = {}.{};\n".format(msg_name, module.id, output))
            pub_name = "pub_" + output_id
            f.write("  {}.publish(&{});\n".format(pub_name, msg_name))
    f.write("}\n")


def generate_firmware(server):
    home = os.path.expanduser("~")
    openag_folder = os.path.join(home, ".openag")
    if not os.path.isdir(openag_folder):
        os.mkdir(openag_folder)
    build_folder = os.path.join(openag_folder, "build")
    if not os.path.isdir(build_folder):
        os.mkdir(build_folder)
    src_path = os.path.join(build_folder, "src")
    lib_path = os.path.join(build_folder, "lib")
    if not os.path.isdir(src_path) or not os.path.isdir(lib_path):
        yes = subprocess.Popen(["yes"], stdout=subprocess.PIPE)
        subprocess.call(
            ["pio", "init", "-d", build_folder, "-b", "megaatmega2560"],
            stdin=yes.stdout
        )
        yes.kill()
        yes.wait()
    ros_lib_path = os.path.join(lib_path, "ros_lib")
    if not os.path.isdir(ros_lib_path):
        if subprocess.call([
            "rosrun", "rosserial_arduino", "make_libraries.py", lib_path
        ]):
            raise RuntimeError("Failed to make rosserial arduino libraries.")

    module_db = server[DbName.FIRMWARE_MODULE]
    module_type_db = server[DbName.FIRMWARE_MODULE_TYPE]

    modules, module_types = read_module_data(
        module_db, module_type_db
    )
    lib_folder = os.path.join(build_folder, "lib")
    download_libs(module_types, lib_folder)
    src_folder = os.path.join(build_folder, "src")
    sketch_file = os.path.join(src_folder, "src.ino")
    with open(sketch_file, "w+") as f:
        write_code(modules, module_types, f)
