from ..base import Plugin

class ROSCommPlugin(Plugin):
    def header_files(self):
        return set([
            "ros.h", "openag_brain/DiagnosticStatus.h",
            "openag_brain/DiagnosticArray.h"
        ])

    def write_declarations(self, f):
        f.writeln("ros::NodeHandle nh;")
        f.writeln("openag_brain::DiagnosticArray status_array;")
        f.writeln(
            'ros::Publisher pub_diagnostics('
                '"/internal_diagnostics", &status_array'
            ');'
        )
        for mod_name, mod_info in self.modules.items():
            # Define publishers for all outputs
            for output_name in mod_info["outputs"]:
                f.writeln(
                    'ros::Publisher {pub_name}('
                        '"{output_topic}", &{msg_name}'
                    ');'.format(
                        pub_name=self.pub_name(mod_name, output_name),
                        output_topic=self.output_topic(mod_name, output_name),
                        msg_name=self.msg_name(mod_name, output_name)
                    )
                )

            # Define callbacks and subscribers for all inputs
            for input_name, input_info in mod_info["inputs"].items():
                cls_name = "::".join(input_info["type"].split("/"))
                arguments = "const {cls_name} &msg".format(cls_name=cls_name)
                callback_name = self.callback_name(mod_name, input_name)
                with f._function("void", callback_name, arguments):
                    f.writeln("{mod_name}.set_{input_name}(msg);".format(
                        mod_name=mod_name, input_name=input_name
                    ))
                f.writeln(
                    'ros::Subscriber<{cls_name}> {sub_name}('
                        '"{input_topic}", {callback_name}'
                    ');'.format(
                        cls_name=cls_name,
                        sub_name=self.sub_name(mod_name, input_name),
                        input_topic=self.input_topic(mod_name, input_name),
                        callback_name=self.callback_name(mod_name, input_name)
                    )
                )

    def setup_plugin(self, f):
        f.writeln("Serial.begin(57600);")
        f.writeln("nh.initNode();")
        f.writeln("nh.advertise(pub_diagnostics);")

    def setup_module(self, mod_name, f):
        mod_info = self.modules[mod_name]
        for output_name in mod_info["outputs"]:
            f.writeln("nh.advertise({pub_name});".format(
                pub_name=self.pub_name(mod_name, output_name)
            ))
        for input_name in mod_info["inputs"]:
            f.writeln("nh.subscribe({sub_name});".format(
                sub_name=self.sub_name(mod_name, input_name)
            ))

    def update_plugin(self, f):
        f.writeln("nh.spinOnce();")

    def update_module(self, mod_name, f):
        f.writeln("nh.spinOnce();")

    def on_output(self, mod_name, output_name, f):
        f.writeln(
            "{pub_name}.publish(&{msg_name});".format(
                pub_name=self.pub_name(mod_name, output_name),
                msg_name=self.msg_name(mod_name, output_name)
            )
        )

    def start_read_module_status(self, f):
        f.writeln(
            "openag_brain::DiagnosticStatus statuses[{num_modules}];".format(
                num_modules = len(self.modules)
            )
        )
        f.writeln("status_array.status_length = {num_modules};".format(
            num_modules=len(self.modules)
        ))
        self.read_module_index = 0

    def read_module_status(self, mod_name, f):
        f.writeln("openag_brain::DiagnosticStatus {mod_name}_status;".format(
            mod_name=mod_name
        ))
        f.writeln("{mod_name}_status.level = {mod_name}.status_level;".format(
            mod_name=mod_name
        ))
        f.writeln('{mod_name}_status.name = "{mod_name}";'.format(
            mod_name=mod_name
        ))
        f.writeln('{mod_name}_status.code = {mod_name}.status_code;'.format(
            mod_name=mod_name
        ))
        f.writeln('statuses[{i}] = {mod_name}_status;'.format(
            i=self.read_module_index, mod_name=mod_name
        ));
        self.read_module_index += 1

    def end_read_module_status(self, f):
        f.writeln("status_array.status = statuses;")
        f.writeln("pub_diagnostics.publish(&status_array);")

    def pub_name(self, mod_name, output_name):
        return "_".join(["pub", mod_name, output_name])

    def sub_name(self, mod_name, input_name):
        return "_".join(["sub", mod_name, input_name])

    def callback_name(self, mod_name, input_name):
        return "_".join([mod_name, input_name, "callback"])

    def output_topic(self, mod_name, output_name):
        return "/sensors/{mod_name}/{output_name}/raw".format(
            mod_name=mod_name, output_name=output_name
        )

    def input_topic(self, mod_name, input_name):
        return "/actuators/{mod_name}/{input_name}".format(
            mod_name=mod_name, input_name=input_name
        )
