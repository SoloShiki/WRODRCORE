#!/usr/bin/env python3
# encoding: utf-8

import rclpy
from rclpy.node import Node
from ros_robot_controller_msgs.msg import SetPWMServoState, PWMServoState
from rcl_interfaces.msg import ParameterDescriptor

class MultiPWMServoController(Node):
    def __init__(self):
        super().__init__('multi_pwm_servo_controller')
        
        # --- 1. DECLARE AND GET PARAMETERS FOR MULTIPLE SERVOS ---
        
        # Define default configurations for the three servos (ID 1, 2, 3)
        # Each element is a list of [servo_id, initial_position, offset, step_size, min_limit, max_limit]
        default_configs = [
            [1, 1000, 0, 100, 500, 2500],  # Servo 1: Wide range, small step
            [2, 2000, 0, 300, 500, 2500],  # Servo 2: Different starting pos, larger step (faster)
            [3, 500, 0, 200, 500, 2500],   # Servo 3: Starts at min limit
        ]

        self.declare_parameter('servo_configs', default_configs, 
                                descriptor=ParameterDescriptor(description='List of [id, initial_pos, offset, step_size, min_limit, max_limit] for each servo.'))
        self.declare_parameter('timer_interval', 0.5)
        
        self.timer_interval = self.get_parameter('timer_interval').get_parameter_value().double_value
        servo_configs = self.get_parameter('servo_configs').get_parameter_value().integer_array_value
        
        # Convert the flat list of parameters into a dictionary for easy state tracking
        self.servo_states = {}
        for config_list in servo_configs:
            servo_id, initial_position, offset, step_size, min_limit, max_limit = config_list
            self.servo_states[servo_id] = {
                'position': initial_position,
                'direction': 1, # 1 for increasing, -1 for decreasing
                'offset': offset,
                'step_size': step_size,
                'min_limit': min_limit,
                'max_limit': max_limit,
            }

        self.publisher = self.create_publisher(SetPWMServoState, '/ros_robot_controller/pwm_servo/set_state', 10)
        self.timer = self.create_timer(self.timer_interval, self.timer_callback)
        
        # Set all servos to their initial positions immediately
        self.set_all_servo_positions()


    # --- 2. MODIFIED set_all_servo_positions TO HANDLE ALL SERVOS ---
    def set_all_servo_positions(self):
        msg = SetPWMServoState()
        log_msg = "Sent positions: "
        
        # Create a single PWMServoState object to hold all commands
        servo_state = PWMServoState()
        
        for servo_id, state in self.servo_states.items():
            position = state['position']
            offset = state['offset']
            
            # Append the ID, position, and offset for the current servo
            servo_state.id.append(servo_id)
            servo_state.position.append(position)
            servo_state.offset.append(offset)
            
            log_msg += f"ID {servo_id} to {position} | "

        # Append the single PWMServoState object (which holds all servo commands)
        msg.state.append(servo_state)
        
        self.publisher.publish(msg)
        self.get_logger().info(log_msg)


    # --- 3. MODIFIED timer_callback TO OSCILLATE ALL SERVOS ---
    def timer_callback(self):
        
        # Update the position and direction for each servo
        for servo_id, state in self.servo_states.items():
            
            current_position = state['position']
            direction = state['direction']
            step_size = state['step_size']
            min_limit = state['min_limit']
            max_limit = state['max_limit']

            # Calculate the new position
            new_position = current_position + direction * step_size
            new_direction = direction
            
            # Check for limits and reverse direction if needed
            if new_position >= max_limit or new_position <= min_limit:
                new_direction *= -1
                # Ensure the position stays within bounds upon reversal
                new_position = max(min_limit, min(new_position, max_limit))
            
            # Update the state dictionary
            self.servo_states[servo_id]['position'] = new_position
            self.servo_states[servo_id]['direction'] = new_direction
            
        # Send the commands for all updated servos
        self.set_all_servo_positions()

def main(args=None):
    rclpy.init(args=args)
    node = MultiPWMServoController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()