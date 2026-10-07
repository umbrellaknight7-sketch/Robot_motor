"""Discrete wrist execution outside the EtherCAT controller and its fault domain."""

import json
import math
import time

import rclpy
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.node import Node
from rclpy.task import Future
from control_msgs.action import FollowJointTrajectory as FJT
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from sensor_msgs.msg import JointState
from std_msgs.msg import Empty, String
from std_srvs.srv import Trigger


NAMES = ('Left_Wrist_Joint', 'Right_Wrist_Joint')


def sample(start, end, fraction):
    u = max(0., min(1., fraction))
    return start + (end - start) * (10*u**3 - 15*u**4 + 6*u**5)


class WristAdapter(Node):
    def __init__(self):
        super().__init__('wheel_human_wrist_adapter')

        def param(name, default):
            return self.declare_parameter(name, default).value

        self.mode = param('mode', 'disabled')
        if self.mode not in ('mock', 'real'):
            raise ValueError('Adapter must be explicitly enabled with mode=mock or real')
        if self.mode == 'real' and not param('commissioned', False):
            raise ValueError('Real wrists require verified calibration, revision and stopping behavior')
        self.timeout = param('feedback_timeout', .5)
        self.heartbeat_timeout = param('heartbeat_timeout', .5)
        self.stop_timeout = param('stop_timeout', 2.)
        self.tolerance = param('goal_tolerance', .02)
        self.velocity_tolerance = param('standstill_velocity', .03)
        self.goal_time = param('goal_time_tolerance', 2.)
        for value in (self.timeout, self.heartbeat_timeout, self.stop_timeout,
                      self.tolerance, self.velocity_tolerance, self.goal_time):
            if not math.isfinite(value) or value <= 0:
                raise ValueError('Timeouts and tolerances must be finite and positive')

        # Keep calibration, SDK sessions and fault status separate for each wrist.
        self.joints = {}
        for name, side in zip(NAMES, ('left', 'right')):
            j = dict(q=None, v=None, received=0., stamp=0., token='', fault='',
                     sdk={}, sdk_time=0., pending=0, stop_ack=False, arm_pending=False)
            for key, default in [('raw_per_radian', 500.), ('raw_zero', 0.), ('lower', -1.),
                                 ('upper', 1.), ('max_velocity', 1.), ('max_acceleration', 3.)]:
                # Defaults are synthetic and are never permitted as real calibration.
                j[key] = param(side + '.' + key, default if self.mode == 'mock' else float('nan'))
                if not math.isfinite(j[key]):
                    raise ValueError('Missing finite calibration: ' + side + '.' + key)
            if (j['raw_per_radian'] == 0 or j['lower'] >= j['upper']
                    or j['max_velocity'] <= 0 or j['max_acceleration'] <= 0):
                raise ValueError('Invalid wrist calibration/limits')
            base = '/cb_' + side
            j['pub'] = self.create_publisher(JointState, base + '_wrist_control_cmd', 1)
            for service in ('arm', 'stop', 'reset'):
                j[service] = self.create_client(Trigger, base + '_wrist/' + service)
            self.create_subscription(JointState, base + '_hand_state',
                                     lambda m, n=name: self.feedback(n, m), 10)
            self.create_subscription(String, base + '_wrist/status',
                                     lambda m, n=name: self.sdk_status(n, m), 10)
            self.create_service(Trigger, '~/' + side + '/reset_fault',
                                lambda req, res, n=name: self.reset(n, res))
            self.joints[name] = j

        self.measured = self.create_publisher(JointState, '/joint_states', 10)
        self.status_pub = self.create_publisher(DiagnosticArray, '~/status', 10)
        self.heartbeat = 0.
        self.create_subscription(Empty, '~/heartbeat', self.beat, 1)
        self.create_service(Trigger, '~/stop', self.stop_service)
        self.action = ActionServer(self, FJT, '~/follow_joint_trajectory', self.execute,
                                   goal_callback=self.accept, cancel_callback=self.cancel)
        self.goal = None
        self.reserved = False
        self.names = []
        self.phase = 'holding'
        self.generation = 0
        self.completion = None
        self.stop_started = 0.
        self.settled = None
        self.result_code = 0
        self.reason = ''
        self.last_status = 0.
        self.create_timer(.02, self.tick)

    def beat(self, _):
        self.heartbeat = time.monotonic()

    def fresh(self, j, now):
        return j['v'] is not None and 0 <= now-j['received'] <= self.timeout

    def available(self, j, now):
        return (now-self.heartbeat <= self.heartbeat_timeout
                and self.fresh(j, now) and not j['fault'] and not j['pending']
                and now-j['sdk_time'] <= self.timeout and j['sdk'].get('active', False)
                and j['sdk'].get('fresh', False) and not j['sdk'].get('fault')
                and not j['sdk'].get('armed', True) and j['arm'].service_is_ready()
                and j['stop'].service_is_ready() and j['lower'] <= j['q'] <= j['upper']
                and abs(j['v']) <= self.velocity_tolerance)

    def feedback(self, name, msg):
        j = self.joints[name]
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        age = self.get_clock().now().nanoseconds * 1e-9 - stamp
        if len(msg.position) != 17 or not math.isfinite(msg.position[16]) or not -0.05 <= age <= self.timeout:
            return
        if stamp <= j['stamp']:
            return

        # Convert only the wrist channel; reject stale samples before estimating velocity.
        q = (msg.position[16] - j['raw_zero']) / j['raw_per_radian']
        delta = stamp - j['stamp']
        velocity = (q-j['q'])/delta if j['q'] is not None and delta <= self.timeout else None
        if not math.isfinite(q) or (velocity is not None and not math.isfinite(velocity)):
            return
        j.update(q=q, v=velocity, stamp=stamp, received=time.monotonic()-max(age, 0.))
        if velocity is not None:
            out = JointState(name=[name], position=[q], velocity=[velocity])
            out.header.stamp = msg.header.stamp
            self.measured.publish(out)

    def sdk_status(self, name, msg):
        try:
            status = json.loads(msg.data)
            if not isinstance(status, dict):
                return
            self.joints[name]['sdk'] = status
            self.joints[name]['sdk_time'] = time.monotonic()
        except (TypeError, ValueError):
            pass

    def accept(self, request):

        def reject(reason):
            if hasattr(self, 'get_logger'):
                self.get_logger().warning('Rejected wrist goal: ' + reason)
            return GoalResponse.REJECT

        now = time.monotonic()
        t = request.trajectory
        if (self.reserved or self.goal or self.phase != 'holding'
                or now-self.heartbeat > self.heartbeat_timeout or not t.joint_names
                or len(set(t.joint_names)) != len(t.joint_names) or len(t.points) != 1
                or t.header.stamp.sec or t.header.stamp.nanosec
                or request.path_tolerance or request.goal_tolerance
                or request.goal_time_tolerance.sec or request.goal_time_tolerance.nanosec):
            return reject("busy/contract/heartbeat; phase=" + self.phase + ", heartbeat_age=" + str(now-self.heartbeat))

        p = t.points[0]
        duration = p.time_from_start.sec + p.time_from_start.nanosec*1e-9
        if (not math.isfinite(duration) or duration <= 0 or duration > 3600
                or len(p.positions) != len(t.joint_names) or p.effort
                or any(not math.isfinite(v) or v != 0 for v in list(p.velocities)+list(p.accelerations))
                or len(p.velocities) not in (0, len(t.joint_names))
                or len(p.accelerations) not in (0, len(t.joint_names))):
            return reject("invalid endpoint, duration, or unsupported derivatives/effort")
        for name, q in zip(t.joint_names, p.positions):
            j = self.joints.get(name)
            if not j or not self.available(j, now) or not math.isfinite(q) or not j['lower'] <= q <= j['upper']:
                return reject("joint unavailable or target outside limits: " + name)
            distance = abs(q-j['q'])
            if 1.875*distance/duration > j['max_velocity'] or 5.774*distance/duration**2 > j['max_acceleration']:
                return reject("duration violates velocity/acceleration limits: " + name)
        self.reserved = True
        return GoalResponse.ACCEPT

    # Wait for SDK arm tokens before the timer starts publishing wrist targets.
    async def execute(self, goal):
        self.reserved = False
        self.goal = goal
        self.names = list(goal.request.trajectory.joint_names)
        p = goal.request.trajectory.points[0]
        self.targets = dict(zip(self.names, p.positions))
        self.starts = {n: self.joints[n]['q'] for n in self.names}
        self.duration = p.time_from_start.sec + p.time_from_start.nanosec*1e-9
        self.completion = Future()
        self.phase = 'arming'
        self.started = time.monotonic()
        self.settled = None
        for n in self.names:
            j = self.joints[n]
            j['arm_pending'] = True
            self.call(n, 'arm', lambda response, name=n: self.armed(name, response))
        result = await self.completion
        if goal.is_cancel_requested and self.result_code == FJT.Result.SUCCESSFUL:
            goal.canceled()
        elif self.result_code == FJT.Result.SUCCESSFUL:
            goal.succeed()
        else:
            goal.abort()
        self.goal = None
        return result

    def call(self, name, service, callback):
        j = self.joints[name]
        if not j[service].service_is_ready():
            callback(None)
            return

        # Pending SDK calls also block reset and stop completion.
        j['pending'] += 1

        def done(future):
            j['pending'] -= 1
            try:
                response = future.result()
            except Exception:
                response = None
            callback(response)

        j[service].call_async(Trigger.Request()).add_done_callback(done)

    def armed(self, name, response):
        j = self.joints[name]
        j['arm_pending'] = False
        if self.phase != 'arming':
            # A late arm must be fenced by another stop before reset/reuse.
            self.stop_one(name)
        elif response and response.success:
            j['token'] = response.message
        else:
            j['fault'] = 'SDK wrist arm failed'
            self.begin_stop(FJT.Result.PATH_TOLERANCE_VIOLATED, j['fault'])

    def cancel(self, _):
        self.begin_stop(0, 'Canceled')
        return CancelResponse.ACCEPT

    def stop_service(self, _, response):
        self.generation += 1
        self.begin_stop(FJT.Result.PATH_TOLERANCE_VIOLATED, 'Stopped by commander')
        response.success, response.message = True, str(self.generation)
        return response

    def stop_one(self, name):
        j = self.joints[name]
        j['token'] = ''
        j['stop_ack'] = False

        def done(response):
            j['stop_ack'] = bool(response and response.success)
            if not j['stop_ack']:
                j['fault'] = 'SDK stop not acknowledged'

        self.call(name, 'stop', done)

    def begin_stop(self, code, reason):
        if self.phase == 'stopping':
            return
        self.phase, self.result_code, self.reason = 'stopping', code, reason
        self.stop_started, self.settled = time.monotonic(), None
        for n in self.names:
            self.stop_one(n)

    def reset(self, name, response):
        j = self.joints[name]
        if self.goal or self.reserved or self.phase != 'holding' or j['pending'] or not self.fresh(j, time.monotonic()) or abs(j['v']) > self.velocity_tolerance:
            response.message = 'Requires idle adapter and fresh stationary wrist feedback; no pending SDK requests'
            return response
        j['fault'] = 'Reset pending'

        def done(result):
            j['fault'] = '' if result and result.success else 'SDK reset failed'

        self.call(name, 'reset', done)
        response.success, response.message = True, 'Reset requested; observe per-joint readiness before retrying'
        return response

    def tick(self):
        now = time.monotonic()
        if self.phase in ('arming', 'moving'):
            for n in self.names:
                j = self.joints[n]
                if not self.fresh(j, now) or now-j['sdk_time'] > self.timeout or j['sdk'].get('fault'):
                    j['fault'] = 'Wrist feedback/SDK unavailable; reset required'
                    self.begin_stop(FJT.Result.PATH_TOLERANCE_VIOLATED, j['fault'])
                    break
            if now-self.heartbeat > self.heartbeat_timeout:
                for n in self.names:
                    self.joints[n]['fault'] = 'Commander heartbeat lost; reset required'
                self.begin_stop(FJT.Result.PATH_TOLERANCE_VIOLATED, 'Commander heartbeat lost')
        if self.phase == 'arming':
            if all(self.joints[n]['token'] for n in self.names):
                self.phase, self.started = 'moving', now
            elif now-self.started > self.stop_timeout:
                self.begin_stop(FJT.Result.PATH_TOLERANCE_VIOLATED, 'SDK arm timeout')

        if self.phase == 'moving':
            elapsed = now-self.started
            desired = []
            for n in self.names:
                j = self.joints[n]
                q = sample(self.starts[n], self.targets[n], elapsed/self.duration)
                desired.append(q)
                msg = JointState(position=[j['raw_zero'] + q*j['raw_per_radian']])
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = j['token']
                j['pub'].publish(msg)
            out = FJT.Feedback(joint_names=self.names)
            out.header.stamp = self.get_clock().now().to_msg()
            out.desired.positions = desired
            out.desired.time_from_start.sec = int(elapsed)
            out.desired.time_from_start.nanosec = int((elapsed % 1)*1e9)
            out.actual.positions = [self.joints[n]['q'] for n in self.names]
            self.goal.publish_feedback(out)
            reached = elapsed >= self.duration and all(
                abs(self.joints[n]['q']-self.targets[n]) <= self.tolerance
                and abs(self.joints[n]['v']) <= self.velocity_tolerance for n in self.names)
            self.settled = (self.settled or now) if reached else None
            if self.settled and now-self.settled >= .1:
                self.begin_stop(0, 'Measured wrist goal reached')
            elif elapsed > self.duration+self.goal_time:
                self.begin_stop(FJT.Result.GOAL_TOLERANCE_VIOLATED, 'Measured wrist goal tolerance exceeded')

        # A stop needs SDK acknowledgement and fresh stationary feedback for every requested wrist.
        if self.phase == 'stopping':
            held = all(self.joints[n]['stop_ack'] and not self.joints[n]['pending']
                       and self.fresh(self.joints[n], now) and self.joints[n]['received'] > self.stop_started
                       and abs(self.joints[n]['v']) <= self.velocity_tolerance for n in self.names)
            self.settled = (self.settled or now) if held else None
            if self.settled and now-self.settled >= .1:
                self.complete()
            elif now-self.stop_started > self.stop_timeout:
                for n in self.names:
                    if not self.joints[n]['stop_ack'] or not self.fresh(self.joints[n], now) or self.joints[n]['pending'] or abs(self.joints[n]['v']) > self.velocity_tolerance:
                        self.joints[n]['fault'] = 'Wrist stop unconfirmed; isolated, reset required'
                self.result_code, self.reason = FJT.Result.PATH_TOLERANCE_VIOLATED, 'Wrist stop unconfirmed; body remains available'
                self.complete()

        if now-self.last_status >= .05:
            self.last_status = now
            msg = DiagnosticArray()
            msg.header.stamp = self.get_clock().now().to_msg()
            for n, j in self.joints.items():
                ready = self.phase == 'holding' and self.available(j, now)
                msg.status.append(DiagnosticStatus(name=n, level=DiagnosticStatus.ERROR if j['fault'] else (DiagnosticStatus.OK if ready else DiagnosticStatus.WARN),
                    message=j['fault'] or (self.phase if self.fresh(j, now) else 'feedback unavailable'),
                    values=[KeyValue(key='ready', value=str(ready).lower()),
                            KeyValue(key='phase', value=self.phase),
                            KeyValue(key='generation', value=str(self.generation))]))
            self.status_pub.publish(msg)

    def complete(self):
        self.phase = 'holding'
        if self.completion and not self.completion.done():
            self.completion.set_result(FJT.Result(error_code=self.result_code, error_string=self.reason))


def main(args=None):
    rclpy.init(args=args)
    node = WristAdapter()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
