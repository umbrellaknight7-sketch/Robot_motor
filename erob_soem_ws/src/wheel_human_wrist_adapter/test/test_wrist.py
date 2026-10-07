import math
from wheel_human_wrist_adapter.node import sample


def test_quintic_endpoint_and_monotonicity():
    assert sample(-.2, .4, 0.) == -.2
    assert math.isclose(sample(-.2, .4, 1.), .4)
    samples = [sample(-.2, .4, i/100.) for i in range(101)]
    assert all(a <= b for a, b in zip(samples, samples[1:]))
    assert (samples[1]-samples[0])/.01 < .001
    assert (samples[-1]-samples[-2])/.01 < .001


def test_goal_contract_rejects_infeasible_mixed_and_unsupported_commands():
    import time
    from types import SimpleNamespace
    from control_msgs.action import FollowJointTrajectory as FJT
    from trajectory_msgs.msg import JointTrajectoryPoint
    from rclpy.action import GoalResponse
    from wheel_human_wrist_adapter.node import WristAdapter
    adapter = SimpleNamespace(reserved=False, goal=None, phase='holding',
        heartbeat=time.monotonic(), heartbeat_timeout=.5,
        joints={'Left_Wrist_Joint': dict(q=0., lower=-1., upper=1., max_velocity=1., max_acceleration=3.)},
        available=lambda *_: True)
    goal = FJT.Goal()
    goal.trajectory.joint_names = ['Left_Wrist_Joint']
    point = JointTrajectoryPoint(positions=[.2])
    point.time_from_start.sec = 2
    goal.trajectory.points = [point]
    assert WristAdapter.accept(adapter, goal) == GoalResponse.ACCEPT
    assert WristAdapter.accept(adapter, goal) == GoalResponse.REJECT
    adapter.reserved = False
    goal.trajectory.points[0].time_from_start.sec = 0
    goal.trajectory.points[0].time_from_start.nanosec = 10000000
    assert WristAdapter.accept(adapter, goal) == GoalResponse.REJECT
    goal.trajectory.points[0].time_from_start.sec = 2
    goal.trajectory.points[0].effort = [1.]
    assert WristAdapter.accept(adapter, goal) == GoalResponse.REJECT
    goal.trajectory.points[0].effort = []
    goal.trajectory.joint_names = ['Feet_Joint']
    assert WristAdapter.accept(adapter, goal) == GoalResponse.REJECT
