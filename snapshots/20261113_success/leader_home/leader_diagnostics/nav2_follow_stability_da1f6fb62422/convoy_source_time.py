"""Strict timing contract for the isolated stabilization candidate."""
import json
import math


def time_issue(stamp, now_ns, max_age=.3):
    if type(stamp) is not int or stamp<=0:
        return 'MISSING_SOURCE_STAMP'
    age=(now_ns-stamp)/1e9
    if age<-.05:return 'SOURCE_IN_FUTURE'
    if age>max_age:return 'SOURCE_STALE'
    return None


def status_payload(text, now_ns, require_measurement=False):
    try:
        data=json.loads(text)
        if not isinstance(data,dict):return None,'INVALID_STATUS'
        issue=time_issue(data.get('status_ros_ns'),now_ns)
        if issue:return None,issue
        if require_measurement:
            issue=time_issue(data.get('source_stamp_ns'),now_ns)
            if issue:return None,issue
        else:
            feedback_age=data.get('feedback_age_s')
            if type(feedback_age) not in (int,float) or not math.isfinite(feedback_age) or feedback_age<0:
                return None,'MISSING_FEEDBACK_AGE'
            status_age=max(0.,(now_ns-data['status_ros_ns'])/1e9)
            if feedback_age+status_age>.3:
                return None,'FEEDBACK_STALE_AT_RECEIPT'
        return data,None
    except (ValueError,TypeError):
        return None,'INVALID_STATUS'
