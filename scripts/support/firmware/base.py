from ..util import dedupe_by, make_dir_name_from_url

def FlowManager(start_string, end_string):
    """
    A factory for creating context managers for standard code constructions,
    such as if statements and functions.

    :param str start_string: A format string for the beginning of this code
    structure
    :param str end_string: A format string for the end of this code structure

    The constructor of the returned class takes as arguments a `CodeWriter`
    instance followed by keyword arguments that will be used to format
    `start_string` and `end_string`
    """
    class Inner(object):
        def __init__(self, f, **kwargs):
            self.f = f
            self.kwargs = kwargs

        def __enter__(self):
            self.f.writeln(start_string.format(**self.kwargs))
            self.f.indent()

        def __exit__(self, type, value, traceback):
            self.f.deindent()
            self.f.writeln(end_string.format(**self.kwargs))
    return Inner

FunctionManager = FlowManager("{type} {name}({args}) {{", "}}")
IfManager = FlowManager("if ({condition}) {{", "}}")
ElseManager = FlowManager("else ({condition}) {{", "}}")
ElIfManager = FlowManager("else if ({condition}) {{", "}}")

class CodeWriter(object):
    """
    Provides a thin wrapper around a file object to make it easy to maintain
    consistent indentation and write some standard code structures when
    programmaticaly generating code.
    """
    def __init__(self, f):
        self.f = f
        self.indent_level = 0

    def writeln(self, data):
        """
        Write a line of text to the file

        :param data: The text to write
        """
        self.f.write(" "*self.indent_level)
        self.f.write(data + "\n")

    def indent(self, levels=1):
        """
        Increase the indent level

        :param int levels: The number by which to increase the indent level
        (defaults to 1)
        """
        self.indent_level += 2*levels

    def deindent(self, levels=1):
        """
        Decrease the indent level

        :param int levels: The number by which to decrease the indent level
        (defaults to 1)
        """
        self.indent_level -= 2*levels
        if self.indent_level < 0:
            raise ValueError("Can't lower the indent level any further")

    def _function(self, type, name, args=""):
        """
        Returns a context manager for writing a function.

        :param str type: The return type of the function
        :param str name: The name of the functino
        :param str args: The argument specification for the function
        """
        return FunctionManager(self, type=type, name=name, args=args)

    def _if(self, condition):
        """
        Returns a context manager for writing an if statement.

        :param str condition: The condition of the if statement
        """
        return IfManager(self, condition=condition)

    def _else(self, condition):
        """
        Returns a context manager for writing an else statement.

        :param str condition: The condition of the else statement
        """
        return ElseManager(self, condition=condition)

    def _elif(self, condition):
        """
        Returns a context manager for writing an else if statement.

        :param str condition: The condition of the else if statement
        """
        return ElIfManager(self, condition=condition)

class Plugin:
    """
    Base class for plugins for the top-level CodeGen class that are responsible
    for generating a particular type of code.
    """
    def __init__(self, modules):
        self.modules = modules

    def pio_dependencies(self):
        """
        Should return a list of PioRepo dicts describing the PlatformIO
        libraries required by this plugin
        """
        return []

    def git_dependencies(self):
        """
        Should return a list of GitRepo dicts describing git repositories
        containing code required by this plugin.
        """
        return []

    def header_files(self):
        """
        Should return a set of all of the header files required for this plugin
        to function
        """
        return set()

    def write_declarations(self, f):
        """
        Should write to `CodeWriter` instance `f` statements that declare
        global objects required by this plugin
        """
        pass

    def setup_plugin(self, f):
        """
        Should write to `CodeWriter` instance `f` statements that set up any
        state required by this plugin
        """
        pass

    def setup_module(self, mod_name, f):
        """
        Should write to `CodeWriter` instance `f` statements that set up any
        state required for the module given by `mod_name`
        """
        pass

    def update_plugin(self, f):
        """
        Should write to `CodeWriter` instance `f` statements that update any
        state used by this plugin
        """
        pass

    def update_module(self, mod_name, f):
        """
        Should write to `CodeWriter` instance `f` statements that update the
        module given by `mod_name`
        """
        pass

    def on_output(self, mod_name, output_name, f):
        """
        Should write to `CodeWriter` instance `f` statements that handle an
        outputs message from the module `mod_name` on the output `output_name`
        """
        pass

    def start_read_module_status(self, f):
        """
        Should write to `CodeWriter` instance `f` statements that handle
        starting the process of reading the statuses of the modules.
        """
        pass

    def read_module_status(self, mod_name, f):
        """
        Should write to `CodeWriter` instance `f` statments that handle the
        current status of the module `mod_name`
        """
        pass

    def end_read_module_status(self, f):
        """
        Should write to `CodeWriter` instance `f` statements that handle
        finishing the process of reading the statuses of the modules.
        """
        pass

    def msg_name(self, mod_name, output_name):
        """
        Returns the name of the message object in the generated code for output
        `output_name` of the module `mod_name`
        """
        return "_".join([mod_name, output_name, "msg"])


class CodeGen(Plugin):
    """
    Class that handles generating code given information about available module
    types and a desired module configuration. Portions of the code that should
    be configurable are provided by plugins.

    `modules` should be a dictionary mapping module names to dictionaries
    validated by the `Module` schema. `module_types` should be a dictionary
    mapping module type names to dictionaries validated by the `ModuleType`
    schema. All other arguments passed in to the constructor will be
    interpretes as plugins. They should instances of subclasses of `Plugin`.
    """
    def __init__(self, modules, plugins, status_update_interval=5):
        self.modules = modules
        self.plugins = (self, ) + tuple(plugins)
        self.status_update_interval = status_update_interval

    def all_pio_dependencies(self):
        deps = []
        for plugin in self.plugins:
            deps.extend(plugin.pio_dependencies())
        for mod_info in self.modules.values():
            if mod_info.get("repository", {}).get("type", None) == "pio":
                deps.append(mod_info["repository"])
            for mod_dep in mod_info.get("dependencies", []):
                if mod_dep["type"] == "pio":
                    deps.append(mod_dep)
        unique_deps = dedupe_by(deps, get_id)
        return unique_deps

    def all_git_dependencies(self):
        deps = []
        for plugin in self.plugins:
            deps.extend(plugin.git_dependencies())
        for mod_info in self.modules.values():
            if mod_info.get("repository", {}).get("type", None) == "git":
                deps.append(mod_info["repository"])
            for mod_dep in mod_info.get("dependencies", []):
                if mod_dep["type"] == "git":
                    deps.append(mod_dep)
        unique_deps = dedupe_by(deps, make_dir_name_from_dep)
        return unique_deps

    def write_to(self, f):
        """
        Generates code based on the given module configuration and writes it to
        the file object `f`.
        """
        f = CodeWriter(f)

        # Write all header files
        headers = set()
        for plugin in self.plugins:
            headers = headers.union(plugin.header_files())
        for header in headers:
            f.writeln("#include <{}>".format(header))
        f.writeln("")

        # Write all declarations
        for plugin in self.plugins:
            plugin.write_declarations(f)
        f.writeln("")

        # Write the setup function
        with f._function("void", "setup"):
            # Setup all plugins
            f.writeln("// Setup all plugins")
            for plugin in self.plugins:
                plugin.setup_plugin(f)
            # Setup all modules
            f.writeln("// Setup all modules")
            for mod_name in self.modules.keys():
                for plugin in self.plugins:
                    plugin.setup_module(mod_name, f)
        f.writeln("")

        # Write the loop function
        with f._function("void", "loop"):
            # Update all plugins
            f.writeln("// Update all plugins")
            for plugin in self.plugins:
                plugin.update_plugin(f)
            # Update all modules
            f.writeln("// Update all modules")
            for mod_name, mod_info in self.modules.items():
                for plugin in self.plugins:
                    plugin.update_module(mod_name, f)

                # Read all module outputs
                for output_name in mod_info["outputs"]:
                    cond = "{mod_name}.get_{output_name}({msg_name})".format(
                        mod_name=mod_name, output_name=output_name,
                        msg_name=self.msg_name(mod_name, output_name)
                    )
                    with f._if(cond):
                        for plugin in self.plugins:
                            plugin.on_output(mod_name, output_name, f)

            # Read statuses of all modules
            f.writeln("// Read statuses of all modules")
            with f._if("should_read_statuses()"):
                for plugin in self.plugins:
                    plugin.start_read_module_status(f)
                for mod_name in self.modules:
                    for plugin in self.plugins:
                        plugin.read_module_status(mod_name, f)
                for plugin in self.plugins:
                    plugin.end_read_module_status(f)

    def git_dependencies(self):
        return [
            {
                "type": "git",
                "url": "https://github.com/OpenAgInitiative/rosserial_arduino_libs.git"
            }
        ]

    def header_files(self):
        res = set()
        for mod_info in self.modules.values():
            res.add(mod_info["header_file"])
        return res

    def write_declarations(self, f):
        # Define the function should_read_statuses
        f.writeln("uint32_t last_status_read = 0;")
        with f._function("bool", "should_read_statuses"):
            f.writeln("uint32_t curr_time = millis();")
            f.writeln("bool res = (curr_time - last_status_read) > {};".format(
                self.status_update_interval*1000
            ))
            with f._if("res"):
                f.writeln("last_status_read = curr_time;")
            f.writeln("return res;")

        for mod_name, mod_info in self.modules.items():
            # Define the module itself
            args_str = ", ".join(
                repr(arg) if not isinstance(arg, bool) else repr(arg).lower()
                for arg in mod_info["arguments"]
            )
            if args_str:
                args_str = "(" + args_str + ")"
            f.writeln("{cls_name} {mod_name}{args};".format(
                cls_name=mod_info["class_name"], mod_name=mod_name,
                args=args_str
            ))

            # Define messages for all outputs
            for output_name, output_info in mod_info["outputs"].items():
                cls_name = "::".join(output_info["type"].split("/"))
                f.writeln("{cls_name} {msg_name};".format(
                    cls_name=cls_name,
                    msg_name=self.msg_name(mod_name, output_name)
                ))

    def setup_module(self, mod_name, f):
        f.writeln("{obj_name}.begin();".format(obj_name=mod_name))

    def update_module(self, mod_name, f):
        f.writeln("{obj_name}.update();".format(obj_name=mod_name))

def make_dir_name_from_dep(dep):
    return make_dir_name_from_url(dep["url"])

def get_id(dep):
    return dep["id"]
