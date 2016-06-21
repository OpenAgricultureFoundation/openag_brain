import os
import time
import rospkg
import requests
from couchdb import PreconditionFailed
from ConfigParser import ConfigParser

from openag_brain import _design
from openag_brain.util import get_or_create, update_doc
from openag_brain.db_names import DbName

def folder_to_dict(path):
    """
    Recursively reads the files from the directory given by `path` and writes
    their contents to a nested dictionary, which is then returned.
    """
    res = {}
    for key in os.listdir(path):
        key_path = os.path.join(path, key)
        if os.path.isfile(key_path):
            val = open(key_path).read()
            key = key.split('.')[0]
            res[key] = val
        else:
            res[key] = folder_to_dict(key_path)
    return res

def init_db(server, hostname):
    """
    Initializes database given by the `couchdb.Server` object `server` assuming
    that the hostname of this machine is given by `hostname`. In particular, it
    sets some couchdb configuration parameters given in the `couchdb.ini` file,
    creates the necessary databases, and pushes design documents to them.
    """
    # Copy the couchdb config file into the correct directory
    rospack = rospkg.RosPack()
    pkg_path = rospack.get_path('openag_brain')
    config_file_path = os.path.join(pkg_path, 'couchdb.ini')
    config = ConfigParser()
    config.read(config_file_path)
    for section in config.sections():
        for param, value in config.items(section):
            url = "{}/_config/{}/{}".format(server.resource.url, section, param)
            current_val = requests.get(url).content.strip()
            desired_val = '"{}"'.format(
                value.replace('"', '\\"').replace("$hostname", hostname)
            )
            if current_val != desired_val:
                res = requests.put(
                    url, data=desired_val
                )
                # Unless there is some delay between requests, CouchDB gets sad.
                # I'm not really sure why
                time.sleep(1)
                if not res.status_code == 200:
                    raise RuntimeError(
                        'Failed to set configuration parameter "{}": {}'.format(
                            param, res.content
                        )
                    )

    # Create all of the databases
    for k,v in DbName.__dict__.items():
        if k.isupper():
            try:
                server.create(v)
            except PreconditionFailed:
                # The db already exists
                pass

    # Push design documents to all of the databases
    design_path = os.path.dirname(_design.__file__)
    for db_name in os.listdir(design_path):
        if db_name.startswith('__'):
            continue
        db_path = os.path.join(design_path, db_name)
        db = server[db_name]
        doc = get_or_create(db, "_design/openag")
        update_doc(doc, folder_to_dict(db_path), db)

