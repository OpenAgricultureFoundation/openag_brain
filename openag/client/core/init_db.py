from .db_names import DbName
from couchdb import Server, PreconditionFailed

if __name__ == '__main__':
    server = Server()
    for db_name in DbName:
        try:
            server.create(db_name.value)
        except PreconditionFailed:
            pass

    # TODO: Create variable entries too
