from .csv import CSVCommPlugin
from .ros import ROSCommPlugin

plugin_map = {
    "csv": CSVCommPlugin,
    "csv_comm": CSVCommPlugin,
    "ros": ROSCommPlugin,
    "ros_comm": ROSCommPlugin
}
