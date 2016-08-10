#!/usr/bin/env python
"""
The `image_persistence.py` module listens for image data from an environment,
converts the images to PNG format and stores them in the CouchDB instance as
attachments to environmental data points. There should be example one instance
of this module for every environment in the system that has camera(s) connected
to it.

It assumes all topics of the type `sensor_msgs/Image` under the namespace
for the environment are streams of images from connected webcams.
"""
import time
import rospy
import requests
from PIL import Image
from couchdb import Server
from StringIO import StringIO
from sensor_msgs.msg import Image as ImageMsg

from openag.cli.config import config as cli_config
from openag.models import EnvironmentalDataPoint
from openag.db_names import ENVIRONMENTAL_DATA_POINT

from openag_brain import params

class ImagePersistence:
    def __init__(self, server, min_update_interval):
        self.db = server[ENVIRONMENTAL_DATA_POINT]
        self.min_update_interval = min_update_interval
        self.namespace = rospy.get_namespace()
        if self.namespace == '/':
            raise RuntimeError(
                "Image persistence module cannot be run in the global "
                "namespace. Please designate an environment for this module."
            )
        self.environment = self.namespace.split('/')[-2]
        self.subscribers = {}
        self.last_update = {}
        # Maps image encodings from ROS format to numpy format
        self.image_format_mapping = {
            "rgb8": "RGB",
            "rgba8": "RGBA",
        }

    def run(self):
        while not rospy.is_shutdown():
            self.update_subscribers()
            time.sleep(5)

    def update_subscribers(self):
        if rospy.is_shutdown():
            return

        for topic, topic_type in rospy.get_published_topics(self.namespace):
            # Ignore anything that is not an image
            if topic_type != "sensor_msgs/Image":
                continue

            # Ignore topics we are already listening to
            if topic in self.subscribers:
                continue

            camera_id = topic.split('/')[-2]
            def callback(item, camera_id=camera_id):
                self.on_image(item, camera_id)
            self.subscribers[topic] = rospy.Subscriber(
                topic, ImageMsg, callback
            )

    def on_image(self, item, camera_id):
        # Rate limit
        curr_time = time.time()
        if (curr_time - self.last_update.get(camera_id, 0)) < \
                self.min_update_interval:
            return
        self.last_update[camera_id] = curr_time

        rospy.logwarn("Posting image")

        image_format = self.image_format_mapping.get(item.encoding, None)
        if image_format is None:
            raise ValueError()
        img = Image.fromstring(
            image_format, (item.width, item.height), item.data
        )
        variable = "image:" + camera_id
        point = EnvironmentalDataPointModel(
            environment=self.environment,
            variable=variable,
            is_desired=False,
            value=None,
            timestamp=time.time()
        )
        point.store(self.db)
        url = "{db_url}/{point_id}/attachment?rev={rev}".format(
            db_url=self.db.resource.url, point_id=point.id, rev=point["_rev"]
        )
        buf = StringIO()
        img.save(buf, "PNG")
        buf.seek(0)
        headers = {
            "Content-Type": "image/png"
        }
        res = requests.put(url, data=buf, headers=headers)
        if res.status_code != 201:
            raise RuntimeError(
                "Failed to post image to database: {}".format(res.content)
            )

if __name__ == '__main__':
    rospy.init_node('image_persistence_1')
    db_server = cli_config["local_server"]["url"]
    if not db_server:
        raise RuntimeError("No database server specified")
    server = Server(db_server)
    try:
        min_update_interval = rospy.get_param("~min_update_interval")
    except KeyError:
        rospy.logwarn(
            "No minimum update interval specified for image persistence module"
        )
        min_update_interval = 60
    mod = ImagePersistence(server, min_update_interval)
    mod.run()
