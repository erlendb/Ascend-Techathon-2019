from __future__ import print_function

import rospy
from cv_bridge import CvBridge
from ascend_msgs.srv import GlobalMap, SingleInspectionReport, TotalInspectionReport
from ascend_msgs.msg import RustReport

def get_windmill_positions():
    """
    Get the position of all windmill in the map.
    Return as a list of positions, with .x .y .z attributes.
    Calling this function should not be done often, as it blocks
    execution while it queries the other nodes.
    """

    mapservice = rospy.ServiceProxy('/GlobalMap', GlobalMap)
    rospy.wait_for_service('/GlobalMap')

    try:
        windmill_map = mapservice()
        return windmill_map.points
    except rospy.ServiceException as e:
        print("Map service error: " + str(e))
        return []

def build_rust_report_message(windmill_position, has_rust, cv2images):
    """
    Create a rust report from the given parameters.
    """
    rust_report = RustReport()
    rust_report.position = windmill_position
    rust_report.has_rust = has_rust

    _cv_bridge = CvBridge()
    rust_report.images = [_cv_bridge.cv2_to_imgmsg(img) for img in cv2images]

    return rust_report

def send_single_inspection_report(rust_report):
    """
    Send a single inspection report. Build the rust report using the
    build_rust_report_message function.
    Used to submit results for task 2, where each windmill should have
    a corresponding inspection report.
    """
    service = rospy.ServiceProxy('/single_inspection_report', SingleInspectionReport)
    response = service(rust_report)
    return response


def send_total_inspection_report(rust_reports):
    """
    Send a total inspection report. Build a rust report using the build_rust_report_message
    function, and then sort the rust reports in a list according to severity, from most severe
    to least severe. Ie. rost_report[0] should describe the windmill with most corrosion/rust.

    Used to submit results for task 3.
    """
    service = rospy.ServiceProxy('/total_inspection_report', TotalInspectionReport)
    response = service(rust_reports)
    return response
