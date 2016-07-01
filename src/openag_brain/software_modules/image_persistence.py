#!/usr/bin/env python
"""
The `image_persistence.py` module listens for image data from an environment,
converts the images to PNG format and stored them in the CouchDB instance as
attachments to environmental data points. There should be example one instance
of this module for every environment in the system that has camera(s) connected
to it.
"""
import time
import rospy
import requests
from PIL import Image
from couchdb import Server
from StringIO import StringIO
from sensor_msgs.msg import Image as ImageMsg

from openag_brain import params
from openag_brain.models import EnvironmentalDataPointModel
from openag_brain.db_names import DbName

class ImagePersistence:
    def __init__(self, server):
        rospy.init_node('image_persistence')
        self.namespace = rospy.get_namespace()
        if self.namespace == '/':
            raise RuntimeError(
                "Image persistence module cannot be run in the global "
                "namespace. Please designate an environment for this module."
            )
        self.environment = self.namespace.split('/')[-2]
        self.db = server[DbName.ENVIRONMENTAL_DATA_POINT]
        self.db_server = server.resource.url
        self.subscribers = {}
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
    server = Server(rospy.get_param('/' + params.DB_SERVER))
    mod = ImagePersistence(server)
    mod.run()
