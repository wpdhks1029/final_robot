"""Small ROS-independent policy for Nav2 goal action status."""

ACTIVE_GOAL_STATUSES = frozenset((1, 2))  # ACCEPTED, EXECUTING; cancellation must not arm.


def has_active_goal(statuses):
    """True only while NavigateToPose reports an accepted/live goal."""
    return any(getattr(status, 'status', None) in ACTIVE_GOAL_STATUSES
               for status in statuses)
