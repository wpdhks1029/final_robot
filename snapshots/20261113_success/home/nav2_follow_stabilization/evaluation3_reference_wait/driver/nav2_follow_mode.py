"""Option policy for the isolated leader-Nav2 integration modes."""

def nav2_leader_mode_issue(*, run, continuous20, dynamic_box, map_rejoin,
                           continue_follow_after_rejoin, follow_only):
    if not run:
        return '--nav2-leader requires --run'
    if follow_only:
        if continuous20 or dynamic_box:
            return '--nav2-leader --follow-only excludes continuous avoidance options'
        return None
    if not (continuous20 and dynamic_box and map_rejoin
            and continue_follow_after_rejoin):
        return '--nav2-leader requires either --follow-only --map-rejoin, or the full continuous avoidance option set'
    return None
