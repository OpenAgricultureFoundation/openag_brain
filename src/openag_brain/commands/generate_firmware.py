import os
import subprocess

from openag_brain.util import pio_build_path
from openag_brain.models import FirmwareModuleTypeModel, FirmwareModuleModel
from openag_brain.db_names import DbName

def read_module_data(module_db, module_type_db):
    """
    Pulls all of the relevant modules and module types from the database.
    Returns 1 dictionary mapping module IDs to `FirmwareModuleModel` instances
    and 1 dictionary mapping module type IDs to `FirmwareModuleTypeModel`
    instances.
    """
    modules = {
        module_id: FirmwareModuleModel.load(module_db, module_id) for module_id
        in module_db if not module_id.startswith('_')
    }
    module_types = {}
    for module in modules.values():
        if not module.type in module_types:
            module_types[module.type] = FirmwareModuleTypeModel.load(
                module_type_db, module.type
            )
        if module_types[module.type] is None:
            raise RuntimeError(
                'Module "{}" references nonexistant module type "{}"'.format(
                    module.id, module.type
                )
            )
    return modules, module_types

def download_libs(module_types):
    """
    Downloads the libraries for all of the module_types from platformio
    """
    for module_type_id, module_type in module_types.items():
        subprocess.call(["pio", "lib", "install", module_type.pio_id])

def write_code(modules, module_types, f):
    """
    Writes firmware code to the file given by the file desriptor `f` using the
    modules given by `modules` and the module types given by `module_types`.
    Assumes the libraries for the module types are already installed (i.e.
    `download_libs` was fun.
    """
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
            parameters_name, "\"" + "\", \"".join(
                str(module.parameters[param]) for param in
                module_type.parameters
            ) + "\""
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
    """
    Generates firmware based on the configuration read from `server`, which
    should be a `couchdb.Server` instance
    """
    build_folder = pio_build_path()
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
            import shutil
            shutil.rmtree(ros_lib_path)
            raise RuntimeError("Failed to make rosserial arduino libraries.")

    module_db = server[DbName.FIRMWARE_MODULE]
    module_type_db = server[DbName.FIRMWARE_MODULE_TYPE]

    modules, module_types = read_module_data(
        module_db, module_type_db
    )
    download_libs(module_types)
    src_folder = os.path.join(build_folder, "src")
    sketch_file = os.path.join(src_folder, "src.ino")
    with open(sketch_file, "w+") as f:
        write_code(modules, module_types, f)
