import rospy
from rosservice import (
    get_service_list, get_service_node, get_service_args, call_service
)
from flask import Flask, jsonify, request

API_VER = '0.0.1'

app = Flask(__name__)

@app.route("/api/{v}/param".format(v=API_VER), methods=['GET'])
def list_params():
    return jsonify(rospy.get_param_names())

@app.route("/api/{v}/param/<path:param_name>".format(v=API_VER), methods=['GET'])
def get_param(param_name):
    return str(rospy.get_param(param_name))

@app.route("/api/{v}/param/<path:param_name>".format(v=API_VER), methods=['POST'])
def set_param(param_name):
    return str(rospy.set_param(param_name, request.values['value']))

@app.route("/api/{v}/service".format(v=API_VER), methods=['GET'])
def list_services():
    return jsonify(get_service_list())

@app.route("/api/{v}/service/<path:service_name>".format(v=API_VER), methods=['GET'])
def get_service_info(service_name):
    service_name = '/' + service_name
    return jsonify({
        "node": get_service_node(service_name),
        "args": get_service_args(service_name)
    })

@app.route("/api/{v}/service/<path:service_name>".format(v=API_VER), methods=['POST'])
def perform_service_call(service_name):
    service_name = '/' + service_name
    args = request.values.to_dict()
    args = {
        k: str(v) if isinstance(v, unicode) else v for k,v in args.items()
    }
    res = call_service(service_name, [args])[1]
    status_code = getattr(res, 'status_code', 200)
    data = getattr(res, 'data', None)
    if data is None:
        data = {k: getattr(res, k) for k in res.__slots__}
    return jsonify(data), status_code
