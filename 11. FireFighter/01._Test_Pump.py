import rclpy
from rclpy.node import Node
from ros_robot_controller_msgs.msg import BusServoPosition 
import time
# Assuming you still need RPi.GPIO for the pump, 
# although in a true ROS system, this should ideally be another topic.
import RPi.GPIO as GPIO

# --- Servo Position Constants (0-1000 scale for 0-240 degrees) ---
# 150 degrees -> 625 
PULSE_150_DEG = 625 
# 170 degrees -> 708 (or 709)
PULSE_170_DEG = 708

# 90 degrees -> 375 (Neutral position)
NEUTRAL_PULSE = 708 

# --- Hardware Setup (Pump Pin) ---
PUMP_PIN = 23
GPIO.setmode(GPIO.BCM)
GPIO.setup(PUMP_PIN, GPIO.OUT)
# Note: In a robust ROS system, the pump state should be managed via a separate service/topic.

class SprayMotionController(Node):
    def __init__(self):
        super().__init__('spray_motion_controller')
        
        # 1. ROS Publisher for the Bus Servo
        self.publisher_ = self.create_publisher(BusServoPosition, 
                                                '/ros_robot_controller/bus_servo/set_position', 
                                                10)
        
        # 2. Motion Timing
        self.cycle_speed = 0.5  # Time in seconds for each move (up/down)
        self.motion_duration = 10.0 # Total duration for the spray motion
        self.servo_id = 1
        
        # 3. State Variables
        self.start_time = self.get_clock().now().to_msg().sec  # Initialize start time
        self.is_up_position = False # Flag to track current servo target
        
        # 4. ROS Timer (controls the frequency of the spraying logic)
        # The timer period should be less than the cycle_speed to ensure timely updates
        timer_period = self.cycle_speed # Adjust to match cycle speed
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.get_logger().info('Spray Motion Controller Node Started. Pump ON.')
        GPIO.output(PUMP_PIN, GPIO.HIGH) # Turn ON water pump immediately on start

    def timer_callback(self):
        current_time = self.get_clock().now().to_msg().sec
        elapsed_time = current_time - self.start_time

        if elapsed_time < self.motion_duration:
            # --- Spray Motion Logic (Alternating between 150 and 170 degrees) ---
            
            # Toggle the target position
            if self.is_up_position:
                target_pulse = PULSE_170_DEG
                self.is_up_position = False
            else:
                target_pulse = PULSE_150_DEG
                self.is_up_position = True

            # --- Publish Servo Command ---
            msg = BusServoPosition()
            msg.id = self.servo_id
            msg.position = target_pulse 
            # Set run_time to the time until the next timer callback (the cycle speed)
            msg.run_time = self.cycle_speed 
            
            self.publisher_.publish(msg)
            self.get_logger().info(f'Spray: ID {msg.id}, Position: {msg.position} ({round((msg.position/1000)*240, 1)} deg)')
            
        else:
            # --- End Motion and Cleanup ---
            self.get_logger().info('Motion duration complete. Pump OFF. Setting servo to neutral.')
            
            # Send final neutral command
            msg = BusServoPosition()
            msg.id = self.servo_id
            msg.position = NEUTRAL_PULSE 
            msg.run_time = 1.0
            self.publisher_.publish(msg)
            
            # Stop Pump and Clean up GPIO
            GPIO.output(PUMP_PIN, GPIO.LOW)
            GPIO.cleanup()
            
            # Stop the ROS timer to exit the loop
            self.timer.cancel()
            self.destroy_node()

def main(args=None):
    rclpy.init(args=args)
    try:
        controller = SprayMotionController()
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass # Allow clean shutdown
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()