#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import Float32, Bool

class TaskManager(Node):
    def __init__(self):
        super().__init__('task_manager')
        
        self.move_pub = self.create_publisher(Point, '/pid_move/goal', 10)
        self.turn_pub = self.create_publisher(Float32, '/pid_turn/goal', 10)
        
        self.create_subscription(Bool, '/pid_move/done', self.action_done_callback, 10)
        self.create_subscription(Bool, '/pid_turn/done', self.action_done_callback, 10)

        # CHANGED: Added Mode Flag to 'move' tuple
        # Format: ('move', distance_meters, axis_mode, heading_lock)
        # axis_mode: 0.0 = Forward/Backward, 1.0 = Left/Right (Strafe)
        # Format: ('turn', angle_degrees)
        self.mission_plan = [
            ('move', 4.0, 0.0, 0.0),   # Move FORWARD 1 meter keeping 0 heading
           # ('turn', 90.0),            # Turn 90 degrees CCW
           # ('turn', -90.0),            # Turn 90 degrees CCW
           # ('move', -1.0, 1.0, 0.0),   # Move FORWARD 1 meter keeping 0 heading
         #   ('move', 0.5, 1.0, 0.0),   # STRAFE 0.5 meters (Y-axis)
         #   ('turn', -90.0)            # Turn 90 degrees CW
        ]
        
        self.current_step = 0
        self.get_logger().info("Task Manager starting mission in 2 seconds...")
        
        self.timer = self.create_timer(2.0, self.start_next_task)

    def action_done_callback(self, msg):
        if msg.data:
            self.get_logger().info("Action finished. Wait 1 second before next task...")
            self.current_step += 1
            self.timer = self.create_timer(1.0, self.start_next_task)

    def start_next_task(self):
        self.timer.cancel()
        
        if self.current_step >= len(self.mission_plan):
            self.get_logger().info("MISSION COMPLETE!")
            return

        task = self.mission_plan[self.current_step]
        action_type = task[0]

        if action_type == 'move':
            goal = Point()
            goal.x = float(task[1]) # Distance
            goal.y = float(task[2]) # Axis Mode (0 or 1)
            goal.z = float(task[3]) # Heading Lock
            
            axis_str = "Forward/Back" if goal.y == 0.0 else "Strafing"
            self.get_logger().info(f"Task {self.current_step+1}: Moving {goal.x}m ({axis_str})")
            self.move_pub.publish(goal)
            
        elif action_type == 'turn':
            goal = Float32()
            goal.data = float(task[1])
            self.get_logger().info(f"Task {self.current_step+1}: Turning {goal.data}deg")
            self.turn_pub.publish(goal)

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(TaskManager())
    rclpy.shutdown()

if __name__ == '__main__':
    main()