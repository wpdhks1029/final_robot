"""Pure remapping policy for the isolated Nav2 convoy launch."""


def remappings_for_nav2_node(node_name):
    """Return per-node routes; never remap the whole Nav2 launch group."""
    rules = [('/tf', 'tf'), ('/tf_static', 'tf_static')]
    if node_name in ('controller_server', 'behavior_server'):
        rules.append(('cmd_vel', '/leader/nav2_cmd_vel'))
    elif node_name == 'velocity_smoother':
        rules.extend([
            ('cmd_vel', '/leader/nav2_cmd_vel'),
            ('cmd_vel_smoothed', '/cmd_vel_smoothed'),
        ])
    return rules
