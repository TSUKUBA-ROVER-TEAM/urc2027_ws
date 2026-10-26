"""Controller recovery ordering without ROS networking or real actuators."""
import importlib.util
from pathlib import Path
import time
from types import MethodType, SimpleNamespace
from unittest.mock import Mock

from controller_manager_msgs.srv import ListControllers, SwitchController
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Time, Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import UInt64MultiArray

spec = importlib.util.spec_from_file_location(
    'recovery_manager', Path(__file__).parents[1] / 'src/recovery_manager.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def manager():
    node = SimpleNamespace()
    for name, method in vars(module.RecoveryManager).items():
        if callable(method) and name != '__init__':
            setattr(node, name, MethodType(method, node))
    now = time.monotonic()
    node.health = {0: False, 1: False, 2: True}
    node.health_time = node.gate_time = now
    node.health_timeout = node.state_timeout = 1.0
    node.positions = {'arm_3_joint': 0.6, 'arm_4_joint': 0.2}
    node.position_times = {joint: now for joint in node.positions}
    node.hardware_joints = ['arm_1_joint', 'arm_2_joint', 'arm_3_joint']
    node.controller_joints = node.hardware_joints + ['arm_4_joint']
    node.required = {2}
    node.controller = 'arm_position_controller'
    node.hardware = 'RobStrideSystem'
    node.state_broadcaster = 'joint_state_broadcaster'
    node.hardware_recovery_pending = False
    node.hardware_error = node.hardware_busy = node.busy = False
    node.phase = 'running'
    node.calibration_paused = False
    node.calibration_busy = False
    node.calibration_verified = False
    node.initial = [0.0, 0.0, 0.6, 0.2]
    node.accept_after = 100
    node.initial_trajectory_duration = .1
    node.gate = (3, 0, 1, 1)
    node.sync_observed_at = None
    node.clock_ns = 1000
    node.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(
        nanoseconds=node.clock_ns,
        to_msg=lambda: Time(sec=node.clock_ns // 1_000_000_000,
                            nanosec=node.clock_ns % 1_000_000_000)))
    node.get_logger = lambda: SimpleNamespace(warning=lambda message: None)
    for name in ['command_publisher', 'running_publisher', 'release_publisher',
                 'recovery_publisher', 'switch_client', 'hardware_client',
                 'hardware_state_client']:
        setattr(node, name, Mock())
    node.switch_client.service_is_ready.return_value = True
    node.hardware_client.service_is_ready.return_value = True
    return node


def result(value):
    return SimpleNamespace(result=lambda: value)


def controllers(active):
    response = ListControllers.Response()
    from controller_manager_msgs.msg import ControllerState
    response.controller = [ControllerState(
        name='arm_position_controller', state='active' if active else 'inactive')]
    return result(response)


def command(stamp):
    msg = JointTrajectory()
    msg.joint_names = ['arm_1_joint', 'arm_2_joint', 'arm_3_joint', 'arm_4_joint']
    msg.points = [JointTrajectoryPoint(positions=[0.0, 0.0, 1.0, 0.2],
                                      time_from_start=Duration(nanosec=100_000_000))]
    msg.header.stamp.nanosec = stamp
    return msg


def test_fast_reconnect_still_requires_stop_then_sync_start_and_initial_command():
    node = manager()
    # Hardware latches an outage even though ready feedback has already returned.
    node.on_gate(UInt64MultiArray(data=[4, 1, 0, 0]))
    assert node.phase == 'stopping'
    node.on_command(command(500))
    node.command_publisher.publish.assert_not_called()
    node._controllers_listed(controllers(True))
    request = node.switch_client.call_async.call_args.args[0]
    assert request.deactivate_controllers == [node.controller]
    assert not request.activate_controllers
    assert request.strictness == SwitchController.Request.STRICT
    # Fresh health by itself cannot bypass the unfinished deactivate service.
    node.gate = (4, 1, 1, 0)
    node._switched(result(SimpleNamespace(ok=True)), False, 4)
    assert node.phase == 'waiting'
    node._controllers_listed(controllers(False))
    request = node.switch_client.call_async.call_args.args[0]
    assert request.activate_controllers == [node.controller]
    node._switched(result(SimpleNamespace(ok=True)), True, 4)
    assert node.phase == 'priming'
    assert list(node.command_publisher.publish.call_args.args[0].points[0].positions) == [0., 0., .6, .2]
    assert node.release_publisher.publish.call_args.args[0].data == 4
    node.on_command(command(500))
    assert node.command_publisher.publish.call_count == 1
    node.on_gate(UInt64MultiArray(data=[4, 0, 1, 1]))
    assert node.phase == 'running'
    # Delayed commands from before/during recovery are dropped after reactivation too.
    node.clock_ns = 2000
    node.on_command(command(999))
    assert node.command_publisher.publish.call_count == 1
    node.on_command(command(1500))
    assert node.command_publisher.publish.call_count == 2


def test_new_outage_during_activation_cannot_release_an_old_epoch():
    node = manager()
    node.phase = 'waiting'
    node.on_gate(UInt64MultiArray(data=[4, 1, 1, 0]))
    node._switched(result(SimpleNamespace(ok=True)), True, 3)
    assert node.phase == 'stopping'
    node.command_publisher.publish.assert_not_called()
    node.release_publisher.publish.assert_not_called()


def test_normal_motor_disconnect_does_not_call_hardware_lifecycle_services():
    node = manager()
    node.health[2] = False
    node.phase = 'waiting'
    node.request_recovery()
    node.hardware_client.call_async.assert_not_called()
    node.hardware_state_client.call_async.assert_not_called()
    assert list(node.recovery_publisher.publish.call_args.args[0].data) == [0, 1, 2]
    node.hardware_error = True
    node.request_recovery()
    node.hardware_client.call_async.assert_called_once()


def test_hardware_lifecycle_is_recovered_only_after_explicit_hardware_error():
    from controller_manager_msgs.msg import HardwareComponentState
    from lifecycle_msgs.msg import State
    node = manager()
    component = HardwareComponentState(name='RobStrideSystem')
    component.state.id = State.PRIMARY_STATE_UNCONFIGURED
    response = result(SimpleNamespace(component=[component]))
    node._hardware_listed(response)
    node.hardware_state_client.call_async.assert_not_called()
    node.hardware_error = True
    node.hardware_busy = True  # Set by request_recovery before listing hardware.
    node._hardware_listed(response)
    request = node.hardware_state_client.call_async.call_args.args[0]
    assert request.name == 'RobStrideSystem'
    assert request.target_state.id == State.PRIMARY_STATE_INACTIVE
    assert node.hardware_busy
    node._hardware_changed(result(SimpleNamespace(ok=True)))
    assert not node.hardware_busy
    component.state.id = State.PRIMARY_STATE_INACTIVE
    node._hardware_listed(response)
    request = node.hardware_state_client.call_async.call_args.args[0]
    assert request.target_state.id == State.PRIMARY_STATE_ACTIVE


def test_position_sync_requires_a_joint_state_received_after_sync_acknowledgement():
    node = manager()
    node.sync_observed_at = time.monotonic()
    assert node._current_targets() is None
    node.on_joint_state(JointState(name=['arm_3_joint', 'arm_4_joint'], position=[.8, .3]))
    assert node._current_targets() == [0., 0., .8, .3]


def test_restart_state_broadcaster_after_error_before_waiting_for_positions():
    from controller_manager_msgs.msg import ControllerState
    node = manager()
    node.hardware_recovery_pending = True
    node.phase = 'waiting'
    response = ListControllers.Response()
    response.controller = [ControllerState(name=node.controller, state='inactive'),
                           ControllerState(name=node.state_broadcaster, state='inactive')]
    node._controllers_listed(result(response))
    request = node.switch_client.call_async.call_args.args[0]
    assert request.activate_controllers == ['joint_state_broadcaster']
    assert not request.deactivate_controllers
    assert node.phase == 'waiting'


def test_no_broadcaster_restart_for_an_ordinary_motor_power_cycle():
    from controller_manager_msgs.msg import ControllerState
    node = manager()
    node.phase = 'stopping'
    response = ListControllers.Response()
    response.controller = [ControllerState(name=node.controller, state='inactive'),
                           ControllerState(name=node.state_broadcaster, state='inactive')]
    node._controllers_listed(result(response))
    node.switch_client.call_async.assert_not_called()


def test_valid_multisegment_trajectory_keeps_timing_and_joint_names():
    node = manager()
    msg = command(500)
    msg.points.append(JointTrajectoryPoint(positions=[0., 0., 1.5, .2],
                                          time_from_start=Duration(nanosec=200_000_000)))
    node.on_command(msg)
    assert node.command_publisher.publish.call_args.args[0] is msg


def test_reject_malformed_nonfinite_or_expired_trajectories():
    import copy
    node = manager()
    valid = command(500)
    messages = []
    msg = copy.deepcopy(valid); msg.points = []; messages.append(msg)
    msg = copy.deepcopy(valid); msg.points[0].positions = [1.]; messages.append(msg)
    msg = copy.deepcopy(valid); msg.points[0].positions[2] = float('nan'); messages.append(msg)
    msg = copy.deepcopy(valid); msg.points[0].velocities = [float('inf')]*4; messages.append(msg)
    msg = copy.deepcopy(valid); msg.points[0].time_from_start.sec = -1; messages.append(msg)
    msg = copy.deepcopy(valid); msg.points.append(copy.deepcopy(msg.points[0])); messages.append(msg)
    msg = copy.deepcopy(valid); msg.points[0].time_from_start = Duration(); messages.append(msg)
    msg = copy.deepcopy(valid); msg.header.stamp = Time(); messages.append(msg)
    msg = copy.deepcopy(valid); msg.header.stamp.nanosec = 2000; messages.append(msg)
    msg = copy.deepcopy(valid); msg.header.stamp.nanosec = 1_000_000_000; messages.append(msg)
    for msg in messages:
        node.on_command(msg)
    node.command_publisher.publish.assert_not_called()


def test_calibration_pause_prevents_commands_recovery_and_auto_activation():
    node = manager()
    node.calibration_paused = True
    node.on_command(command(500))
    node.command_publisher.publish.assert_not_called()
    node.request_recovery()
    node.recovery_publisher.publish.assert_not_called()
    node.hardware_client.call_async.assert_not_called()
    node._controllers_listed(controllers(True))
    assert node.switch_client.call_async.call_args.args[0].deactivate_controllers == [node.controller]
    node.switch_client.reset_mock()
    node._controllers_listed(controllers(False))
    assert node.phase == 'waiting'
    node.switch_client.call_async.assert_not_called()
    node.phase = 'priming'
    node.on_gate(UInt64MultiArray(data=[3, 0, 1, 1]))
    assert node.phase != 'running'


def test_calibration_blocks_late_activation_and_requires_explicit_verified_resume():
    from std_srvs.srv import Trigger
    node = manager()
    node.calibration_paused = True
    node._switched(result(SimpleNamespace(ok=True)), True, 3)
    node.release_publisher.publish.assert_not_called()
    response = node.resume_calibration(Trigger.Request(), Trigger.Response())
    assert not response.success
    assert node.calibration_paused
    node.calibration_verified = True
    response = node.resume_calibration(Trigger.Request(), Trigger.Response())
    assert response.success
    assert not node.calibration_paused
    assert node.phase == 'waiting'
    assert not node.calibration_verified
    node.release_publisher.publish.assert_not_called()  # Normal handshake still required.


def test_zero_workflow_sends_only_after_stop_and_remains_paused_on_success_or_failure():
    import asyncio
    from std_srvs.srv import Trigger
    for success in [True, False]:
        node = manager()
        node.zero_clients = {'arm_3_joint': Mock()}
        node.zero_clients['arm_3_joint'].call_async.return_value = SimpleNamespace(
            done=lambda: True,
            result=lambda: Trigger.Response(success=success, message='driver result'))
        waits = []

        async def wait_for(predicate, timeout):
            waits.append(timeout)
            if len(waits) == 1:
                assert node.calibration_paused
                assert node.accept_after is None
                node._controllers_listed(controllers(False))
                node.gate = (4, 1, 1, 0)
            if len(waits) == 4:
                node.on_joint_state(JointState(name=['arm_3_joint'], position=[0.0]))
            assert predicate()
            return True

        node._wait_for = wait_for
        response = asyncio.run(node.set_joint_zero(
            'arm_3_joint', Trigger.Request(), Trigger.Response()))
        assert response.success == success
        assert node.calibration_paused
        assert not node.calibration_busy
        assert node.calibration_verified == success
        node.zero_clients['arm_3_joint'].call_async.assert_called_once()
        node.release_publisher.publish.assert_not_called()


def test_zero_workflow_does_not_send_can_request_if_stop_unconfirmed():
    import asyncio
    from std_srvs.srv import Trigger
    node = manager()
    node.zero_clients = {'arm_3_joint': Mock()}

    async def wait_for(predicate, timeout):
        return False

    node._wait_for = wait_for
    response = asyncio.run(node.set_joint_zero(
        'arm_3_joint', Trigger.Request(), Trigger.Response()))
    assert not response.success
    assert node.calibration_paused
    node.zero_clients['arm_3_joint'].call_async.assert_not_called()
