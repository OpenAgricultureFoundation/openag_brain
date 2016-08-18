#!/usr/bin/env python
"""
Stitches together the images from a recipe run and stores them as a video on
the recipe_start data point
"""
import time
from openag.cli.config import config as cli_config
from openag.couch import Server
from openag.db_names import ENVIRONMENTAL_DATA_POINT
from openag.var_types import RECIPE_START, RECIPE_END, AERIAL_IMAGE
import rospy
import tempfile

class VideoWriter(object):
    def __init__(self, server, environment, variable):
        self.image_dir = tempfile.mkdtemp()
        self.data_db = server[ENVIRONMENTAL_DATA_POINT]
        self.environment = environment
        self.variable = variable
        self.start_doc = None
        self.end_doc = None

        # Initialize change feeds
        self.last_seq_by_var = {}
        last_db_seq = self.data_db.changes(limit=1, descending=True)
        for var in [RECIPE_START, RECIPE_END, self.variable]:
            self.last_seq_by_var[var] = last_db_seq

        # Figure out when the most recent recipe started
        start_view = self.data_db.view("openag/by_variable", startkey=[
            self.environment, RECIPE_START.name, "desired"
        ], endkey=[
            self.environment, RECIPE_START.name, "desired", {}
        ], group_level=3)
        if len(start_view) == 0:
            # There is no recipe running
            return
        self.start_doc = start_view.rows[0].value

        # Make sure the recipe hasn't ended yet
        end_view = self.data_db.view("openag/by_variable", startkey=[
            self.environment, RECIPE_END.name, "desired"
        ], endkey=[
            self.environment, RECIPE_END.name, "desired", {}
        ], group_level=3)
        if len(end_view):
            end_doc = end_view.rows[0].value
            if (end_doc["timestamp"] > self.start_doc["timestamp"]):
                self.start_doc = None
                return

        # Download all of the images from the recipe run so far
        image_view = self.data_db.view("openag/by_variable", startkey=[
            self.environment, AERIAL_IMAGE.name, "measured",
            self.start_doc["timestamp"]
        ], endkey=[
            self.environment, AERIAL_IMAGE.name, "measured", {}
        ])
        for row in image_view:
            self.download_image(row.value)

        self.update_video()

    def run(self):
        while True:
            time.sleep(60)
            res = get_variable_changes(RECIPE_END)
            if len(res):
                self.end_doc = res[-1]
            res = get_variable_changes(RECIPE_START)
            if len(res):
                self.start_doc = res[-1]
            if self.start_doc["timestamp"] > self.end_doc["timestamp"]:
                # A recipe is running
                res = get_variable_changes(self.variable)
                for x in res:
                    if x["timestamp"] > self.start_doc["timestamp"]:
                        self.download_image(x)
                if len(res):
                    self.update_video()

    def get_variable_changes(self, variable):
        """
        Get a list of all new environmental data points of the given variable
        since the last time this function was called with that variable
        """
        changes = self.data_db.changes(
            since=self.last_seq_by_var.get(variable, 0),
            filter="openag/by_variable", variable=variable
        )
        self.last_seq_by_variable[variable] = changes["last_seq"]
        return changes["results"]

    def download_image(self, doc):
        """
        Downloads the image stored as a attachment on the given document and
        stores it in the folder with the rest of the images for the current
        recipe run
        """
        raise NotImplementedError()

    def update_video(self):
        """
        Constructs a video from the images already downloaded and stored it in
        the RECIPE_START document for the current recipe run
        """
        raise NotImplementedError()

if __name__ == '__main__':
    db_server = cli_config["local_server"]["url"]
    if not db_server:
        raise RuntimeError("No local database specified")
    server = Server(db_server)
    rospy.init_node("video_writer")
    namespace = rospy.get_namespace()
    if namespace == '/':
        raise RuntimeError(
            "Video writer cannot be run in the global namespace. Please "
            "designate an environment for this module."
        )
    environment = namespace.split('/')[-2]
    mod = VideoWriter(server, environment, AERIAL_IMAGE)
    mod.run()
