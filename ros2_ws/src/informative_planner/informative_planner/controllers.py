"""Seleção do executor de waypoints (Nav2 ou MRS) por parâmetro."""

from informative_planner import config

VALID_CONTROLLER_BACKENDS = ('nav2', 'mrs')


def declare_controller_parameters(node, default_odom_topic='/odom'):
    """Declare the backend parameters on `node` and return their values."""
    backend = str(
        node.declare_parameter(
            'controller_backend',
            config.CONTROLLER_BACKEND,
        ).value or config.CONTROLLER_BACKEND
    ).strip().lower()
    if backend not in VALID_CONTROLLER_BACKENDS:
        raise ValueError(
            f'controller_backend invalido: {backend!r}. '
            f'Use um de {VALID_CONTROLLER_BACKENDS}.'
        )
    navigate_action = str(
        node.declare_parameter(
            'navigate_action',
            '/navigate_to_pose',
        ).value or '/navigate_to_pose'
    )
    odom_topic = str(
        node.declare_parameter('odom_topic', default_odom_topic).value
        or default_odom_topic
    )
    return backend, navigate_action, odom_topic


def build_controller(backend, robot_name, navigate_action, odom_topic):
    """Create the waypoint executor node for the selected backend."""
    if backend == 'nav2':
        from informative_planner.nav2_controller import Nav2Controller
        return Nav2Controller(
            robot_name=robot_name,
            action_name=navigate_action,
            odom_topic=odom_topic,
        )
    # Import tardio: mrs_msgs/mrs_modules_msgs só são exigidos no backend MRS.
    from informative_planner.drone_controler import DroneController
    return DroneController(uav_name=robot_name)
