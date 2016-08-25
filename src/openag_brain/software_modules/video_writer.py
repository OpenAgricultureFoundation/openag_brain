#!/usr/bin/env python
"""
Stitches together the images from a recipe run and stores them as a video on
the recipe_start data point
"""
import os
import time
import rospy
import tempfile
import subprocess
from openag.cli.config import config as cli_config
from openag.couch import Server
from openag.db_names import ENVIRONMENTAL_DATA_POINT
from openag.var_types import RECIPE_START, RECIPE_END, AERIAL_IMAGE

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
        last_db_seq = self.data_db.changes(
            limit=1, descending=True
        )['last_seq']
        for var in [RECIPE_START, RECIPE_END, self.variable]:
            self.last_seq_by_var[var] = last_db_seq

        # Figure out when the most recent recipe started
        start_view = self.data_db.view("openag/by_variable", startkey=[
            self.environment, "desired", RECIPE_START.name
        ], endkey=[
            self.environment, "desired", RECIPE_START.name, {}
        ], group_level=3)
        if len(start_view) == 0:
            # No recipe has ever been run on this machine
            return
        self.start_doc = start_view.rows[0].value

        # Make sure the recipe hasn't ended yet
        end_view = self.data_db.view("openag/by_variable", startkey=[
            self.environment, "desired", RECIPE_END.name
        ], endkey=[
            self.environment, "desired", RECIPE_END.name, {}
        ], group_level=3)
        if len(end_view):
            self.end_doc = end_view.rows[0].value
            if (self.end_doc["timestamp"] > self.start_doc["timestamp"]):
                return

        # Download all of the images from the recipe run so far
        image_view = self.data_db.view("openag/by_variable", startkey=[
            self.environment, "measured", AERIAL_IMAGE.name,
            self.start_doc["timestamp"]
        ], endkey=[
            self.environment, "measured", AERIAL_IMAGE.name, {}
        ], group_level=4)
        for row in image_view:
            self.download_image(row.value)
        self.update_video()

    def __del__(self):
        import shutil
        shutil.rmtree(self.image_dir)

    def run(self):
        while True:
            time.sleep(5)
            if rospy.is_shutdown():
                break
            if self.start_doc and (not self.end_doc or self.start_doc["timestamp"] > self.end_doc["timestamp"]):
                # A recipe is running
                # Check if it has ended
                end_docs = self.get_variable_changes(RECIPE_END)
                for end_doc in end_docs:
                    if end_doc["timestamp"] > self.end_doc["timestamp"]:
                        self.end_doc = end_doc
                # Update the timelapse
                res = self.get_variable_changes(self.variable)
                should_update_video = False
                for img in res:
                    if img["timestamp"] > self.start_doc["timestamp"]:
                        self.download_image(img)
                        should_update_video = True
                if should_update_video:
                    self.update_video()
            else:
                # No recipe is running
                # Check if a recipe has started
                res = self.get_variable_changes(RECIPE_START)
                if len(res):
                    self.start_doc = res[-1]

    def get_variable_changes(self, variable):
        """
        Get a list of all new environmental data points of the given variable
        since the last time this function was called with that variable
        """
        res = self.data_db.changes(
            since=self.last_seq_by_var.get(variable, 0),
            filter="openag/by_variable", variables=[variable],
            include_docs=True
        )
        self.last_seq_by_var[variable] = res["last_seq"]
        return [x["doc"] for x in res["results"]]

    def download_image(self, doc):
        """
        Downloads the image stored as a attachment on the given document and
        stores it in the folder with the rest of the images for the current
        recipe run
        """
        image = self.data_db.get_attachment(doc, "image")
        if image is None:
            # We might see the document before the attachment is uploaded. Wait
            # a little while and try again
            time.sleep(1)
            image = self.data_db.get_attachment(doc, "image")
        file_name = str(int(doc["timestamp"])) + ".png"
        file_path = os.path.join(self.image_dir, file_name)
        with open(file_path, "w+") as f:
            f.write(image.read())

    def update_video(self):
        """
        Constructs a video from the images already downloaded and stores it in
        the RECIPE_START document for the current recipe run
        """
        out_file = os.path.join(self.image_dir, "out.mp4")
        if os.path.isfile(out_file):
            os.remove(out_file)
        if subprocess.call([
            "ffmpeg", "-framerate", "1", "-pattern_type", "glob", "-i",
            "*.png", "-c:v", "libx264", "out.mp4"
        ], cwd=self.image_dir):
            raise RuntimeError("Failed to update video")
        with open(out_file) as f:
            print self.data_db.put_attachment(
                self.start_doc, f, "timelapse", "video/mp4"
            )

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
