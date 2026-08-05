#!/usr/bin/env python3

import sys

import rospy
from std_srvs.srv import Trigger


def call_trigger(service_name: str) -> None:
    rospy.loginfo("Waiting for service: %s", service_name)

    rospy.wait_for_service(
        service_name,
        timeout=20.0
    )

    service = rospy.ServiceProxy(
        service_name,
        Trigger
    )

    response = service()

    if not response.success:
        raise RuntimeError(
            "{} failed: {}".format(
                service_name,
                response.message
            )
        )

    rospy.loginfo(
        "%s succeeded: %s",
        service_name,
        response.message
    )


def main() -> None:
    rospy.init_node("prepare_danger_result_writer")

    reset_results = rospy.get_param(
        "~reset_results",
        True
    )

    start_recording = rospy.get_param(
        "~start_recording",
        True
    )

    try:
        if reset_results:
            call_trigger(
                "/danger_result_writer/reset"
            )

        if start_recording:
            call_trigger(
                "/danger_result_writer/start"
            )

    except Exception as error:
        rospy.logerr(
            "Failed to prepare result writer: %s",
            error
        )
        sys.exit(1)

    rospy.loginfo(
        "Result writer preparation complete."
    )


if __name__ == "__main__":
    main()
