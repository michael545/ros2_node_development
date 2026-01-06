from launch import LaunchDescription
from launch.actions import EmitEvent, RegisterEventHandler
from launch.events import matches_action
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
import lifecycle_msgs.msg


def generate_launch_description():
    lifecycle_node = LifecycleNode(
        package='lifecycle_example',
        executable='lifecycle_talker',
        name='lifecycle_talker_node',
        namespace='',
        output='screen'
    )

    # When the lifecycle_talker node reaches the 'inactive' state, trigger a transition to 'active'.
    register_event_handler_for_inactive_to_active = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=lifecycle_node,
            goal_state='inactive',
            entities=[
                EmitEvent(event=ChangeState(
                    lifecycle_node_matcher=matches_action(lifecycle_node),
                    transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
                )),
            ],
        )
    )

    # When the lifecycle_talker node reaches the 'unconfigured' state, trigger a transition to 'configuring'.
    register_event_handler_for_unconfigured_to_configuring = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=lifecycle_node,
            goal_state='unconfigured',
            entities=[
                EmitEvent(event=ChangeState(
                    lifecycle_node_matcher=matches_action(lifecycle_node),
                    transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
                )),
            ],
        )
    )

    return LaunchDescription([
        lifecycle_node,
        register_event_handler_for_unconfigured_to_configuring,
        register_event_handler_for_inactive_to_active,
    ])