import rospy
import rosservice
from flask import Flask, jsonify, request

API_VER = '0.0.1'

app = Flask(__name__)
app.debug = True

@app.route("/api/{v}/param".format(v=API_VER), methods=['GET'])
def list_params():
    return jsonify({"results": rospy.get_param_names()})

@app.route("/api/{v}/param/<path:param_name>".format(v=API_VER), methods=['GET'])
def get_param(param_name):
    if not rospy.has_param(param_name):
        return jsonify({"error": "No such parameter exists"}), 400
    return jsonify({"result": str(rospy.get_param(param_name))})

@app.route("/api/{v}/param/<path:param_name>".format(v=API_VER), methods=['POST'])
def set_param(param_name):
    if not 'value' in request.values:
        return jsonify({"error": "No value supplied"}), 400
    rospy.set_param(param_name, request.values['value'])
    return "", 204

@app.route("/api/{v}/service".format(v=API_VER), methods=['GET'])
def list_services():
    return jsonify({"results": rosservice.get_service_list()})

@app.route("/api/{v}/service/<path:service_name>".format(v=API_VER), methods=['GET'])
def get_service_info(service_name):
    service_name = '/' + service_name
    service_type = rosservice.get_service_type(service_name)
    if not service_type:
        return jsonify({"error": "No such service exists"}), 404
    return jsonify({
        "request_type": service_type,
        "node": rosservice.get_service_node(service_name),
        "args": rosservice.get_service_args(service_name)
    })

@app.route("/api/{v}/service/<path:service_name>".format(v=API_VER), methods=['POST'])
def perform_service_call(service_name):
    service_name = '/' + service_name
    args = request.values.to_dict()
    args = {
        k: str(v) if isinstance(v, unicode) else v for k,v in args.items()
    }
    try:
        res = rosservice.call_service(service_name, [args])[1]
    except rosservice.ROSServiceException as e:
        return jsonify({"error": str(e)}), 400
    status_code = getattr(res, 'status_code', 200)
    data = getattr(res, 'data', None)
    if data is None:
        data = {k: getattr(res, k) for k in res.__slots__}
    return jsonify(data), status_code
