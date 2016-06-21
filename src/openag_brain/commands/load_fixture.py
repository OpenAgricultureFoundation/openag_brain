import os
import json

from openag_brain import fixtures
from openag_brain.util import get_or_create, update_doc

def load_fixture(server, fixture_name):
    """
    Given a `couchdb.Server` instance `server` and a `fixture_name`, this
    function reads in the fixture and creates the corresponding objects on the
    server.
    """
    fixture_file_name = os.path.join(
        os.path.dirname(fixtures.__file__), fixture_name + ".json"
    )
    with open(fixture_file_name) as fixture_file:
        fixture = json.load(fixture_file)
    for db_name, items in fixture.items():
        db = server[db_name]
        for item in items:
            item_id = item.pop("_id")
            doc = get_or_create(db, item_id)
            update_doc(doc, item, db)

