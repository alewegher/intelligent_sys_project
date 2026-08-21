#!/usr/bin/env python3
"""
Stops the whole run after a fixed amount of SIMULATION time.

Why not a launch TimerAction: launch timers run on the wall clock, so if Gazebo's
real-time factor drifts (CPU load, another process, a heavier configuration) two runs of
the same nominal length would record different amounts of SIMULATED time - different
numbers of trajectory laps, different sample counts, and therefore metrics that are not
comparable across runs. Since every analysis works in sim time (header.stamp), the run
length must be defined in sim time too.

This node's timer runs on the node clock with use_sim_time:=True, i.e. it ticks on /clock.
It exits cleanly when the budget is spent; the launch file turns that exit into a Shutdown
event, which SIGINTs `ros2 bag record` so the bag is closed properly.
"""

import rclpy
from rclpy.node import Node


class RunTimer(Node):
    def __init__(self):
        super().__init__('run_timer')
        self.duration = self.declare_parameter('duration', 0.0).value
        self.t0 = None
        self.done = False
        # 0.5 s is plenty: it bounds the overshoot past the requested duration, and the
        # bag is trimmed by header.stamp in analysis anyway.
        self.create_timer(0.5, self.check)
        self.get_logger().info(
            'Run timer waiting for /clock (budget: %.1f s of simulation time)' % self.duration)

    def check(self):
        if self.done:
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        if now <= 0.0:
            return  # no /clock yet - sim time still zero
        if self.t0 is None:
            self.t0 = now
            self.get_logger().info(
                'Run timer armed at sim t=%.2f s: will stop the run at t=%.2f s'
                % (self.t0, self.t0 + self.duration))
            return
        elapsed = now - self.t0
        if elapsed >= self.duration:
            self.done = True
            self.get_logger().info(
                'Reached %.1f s of simulation time - shutting the run down' % elapsed)
            raise SystemExit


def main():
    rclpy.init()
    node = RunTimer()
    try:
        rclpy.spin(node)
    except (SystemExit, KeyboardInterrupt):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
