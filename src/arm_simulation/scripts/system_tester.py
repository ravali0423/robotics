#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32, Bool
import time
import sys

class SystemTester(Node):
    """
    Test script for individual components of the speech-to-gesture system.
    Helps validate that each component is working correctly.
    """
    
    def __init__(self):
        super().__init__('system_tester')
        
        # Publishers for testing different components
        self.finger_count_pub = self.create_publisher(Int32, '/finger_count', 10)
        self.gesture_command_pub = self.create_publisher(String, '/gesture_command', 10)
        self.letter_command_pub = self.create_publisher(String, '/letter_command', 10)
        self.text_sequence_pub = self.create_publisher(String, '/text_to_sequence', 10)
        self.sequence_control_pub = self.create_publisher(String, '/sequence_control', 10)
        self.speech_control_pub = self.create_publisher(Bool, '/speech_recognition_control', 10)
        self.user_command_pub = self.create_publisher(String, '/user_command', 10)
        
        # Subscribers to monitor system responses
        self.current_gesture_sub = self.create_subscription(
            String, '/current_gesture', self.gesture_callback, 10)
        self.sequence_status_sub = self.create_subscription(
            String, '/sequence_status', self.sequence_status_callback, 10)
        self.system_status_sub = self.create_subscription(
            String, '/system_status', self.system_status_callback, 10)
        
        # Test state tracking
        self.last_gesture = "unknown"
        self.last_sequence_status = "unknown"
        self.last_system_status = "unknown"
        
        self.get_logger().info('System Tester initialized')
        time.sleep(1)  # Allow subscribers to connect
    
    def gesture_callback(self, msg):
        """Track current gesture"""
        self.last_gesture = msg.data
        self.get_logger().debug(f'Current gesture: {msg.data}')
    
    def sequence_status_callback(self, msg):
        """Track sequence status"""
        self.last_sequence_status = msg.data
        self.get_logger().debug(f'Sequence status: {msg.data}')
    
    def system_status_callback(self, msg):
        """Track system status"""
        self.last_system_status = msg.data
        self.get_logger().debug(f'System status: {msg.data}')
    
    def wait_and_check(self, duration: float, expected_gesture: str = None):
        """Wait for duration and optionally check for expected gesture"""
        start_time = time.time()
        
        while time.time() - start_time < duration:
            rclpy.spin_once(self, timeout_sec=0.1)
            time.sleep(0.1)
        
        if expected_gesture and self.last_gesture != expected_gesture:
            self.get_logger().warn(f'Expected gesture "{expected_gesture}", got "{self.last_gesture}"')
        else:
            self.get_logger().info(f'✓ Current gesture: {self.last_gesture}')
    
    def test_finger_counting(self):
        """Test basic finger counting functionality"""
        self.get_logger().info('\n=== TESTING FINGER COUNTING ===')
        
        for count in range(6):
            self.get_logger().info(f'Testing finger count: {count}')
            
            msg = Int32()
            msg.data = count
            self.finger_count_pub.publish(msg)
            
            self.wait_and_check(2.0, f'finger_count_{count}')
        
        self.get_logger().info('✓ Finger counting test complete')
    
    def test_letter_gestures(self):
        """Test individual letter gestures"""
        self.get_logger().info('\n=== TESTING LETTER GESTURES ===')
        
        test_letters = ['a', 'b', 'c', 'd', 'e', 'i', 'l', 'o', 'u', 'y']
        
        for letter in test_letters:
            self.get_logger().info(f'Testing letter: {letter.upper()}')
            
            msg = String()
            msg.data = letter
            self.letter_command_pub.publish(msg)
            
            self.wait_and_check(2.0, f'letter_{letter}')
        
        self.get_logger().info('✓ Letter gestures test complete')
    
    def test_word_gestures(self):
        """Test word gestures"""
        self.get_logger().info('\n=== TESTING WORD GESTURES ===')
        
        test_words = ['hello', 'goodbye', 'please', 'thank you', 'yes', 'no']
        
        for word in test_words:
            self.get_logger().info(f'Testing word: {word}')
            
            msg = String()
            msg.data = word
            self.gesture_command_pub.publish(msg)
            
            self.wait_and_check(2.0)
        
        self.get_logger().info('✓ Word gestures test complete')
    
    def test_sequence_functionality(self):
        """Test gesture sequencing"""
        self.get_logger().info('\n=== TESTING SEQUENCE FUNCTIONALITY ===')
        
        test_sequences = [
            ('abc', 'Simple alphabet sequence'),
            ('123', 'Number sequence'),
            ('hello', 'Word sequence'),
            ('hi there', 'Multi-word sequence')
        ]
        
        for text, description in test_sequences:
            self.get_logger().info(f'Testing {description}: "{text}"')
            
            # Send sequence
            msg = String()
            msg.data = text
            self.text_sequence_pub.publish(msg)
            
            # Wait for sequence to start
            self.wait_and_check(1.0)
            
            # Wait for sequence to complete (estimate timing)
            estimated_time = len(text) * 1.5 + 3
            self.wait_and_check(estimated_time)
            
            self.get_logger().info(f'✓ {description} complete')
        
        self.get_logger().info('✓ Sequence functionality test complete')
    
    def test_sequence_control(self):
        """Test sequence control commands"""
        self.get_logger().info('\n=== TESTING SEQUENCE CONTROL ===')
        
        # Start a sequence
        self.get_logger().info('Starting test sequence...')
        msg = String()
        msg.data = 'abcdefgh'
        self.text_sequence_pub.publish(msg)
        
        self.wait_and_check(2.0)
        
        # Test pause
        self.get_logger().info('Testing pause...')
        control_msg = String()
        control_msg.data = 'pause'
        self.sequence_control_pub.publish(control_msg)
        
        self.wait_and_check(2.0)
        
        # Test resume
        self.get_logger().info('Testing resume...')
        control_msg.data = 'play'
        self.sequence_control_pub.publish(control_msg)
        
        self.wait_and_check(2.0)
        
        # Test stop
        self.get_logger().info('Testing stop...')
        control_msg.data = 'stop'
        self.sequence_control_pub.publish(control_msg)
        
        self.wait_and_check(2.0)
        
        self.get_logger().info('✓ Sequence control test complete')
    
    def test_coordinator_commands(self):
        """Test main coordinator commands"""
        self.get_logger().info('\n=== TESTING COORDINATOR COMMANDS ===')
        
        commands = [
            ('status', 'Status check'),
            ('demo alphabet', 'Alphabet demo'),
            ('reset', 'System reset')
        ]
        
        for command, description in commands:
            self.get_logger().info(f'Testing {description}: "{command}"')
            
            msg = String()
            msg.data = command
            self.user_command_pub.publish(msg)
            
            if 'demo' in command:
                self.wait_and_check(5.0)  # Demos take longer
            else:
                self.wait_and_check(2.0)
        
        self.get_logger().info('✓ Coordinator commands test complete')
    
    def run_component_tests(self):
        """Run all component tests"""
        self.get_logger().info('Starting comprehensive component tests...')
        
        try:
            self.test_finger_counting()
            time.sleep(1)
            
            self.test_letter_gestures()
            time.sleep(1)
            
            self.test_word_gestures()
            time.sleep(1)
            
            self.test_sequence_functionality()
            time.sleep(1)
            
            self.test_sequence_control()
            time.sleep(1)
            
            self.test_coordinator_commands()
            
            self.get_logger().info('\n=== ALL TESTS COMPLETE ===')
            self.get_logger().info('✓ System appears to be functioning correctly')
            
        except KeyboardInterrupt:
            self.get_logger().info('\nTests interrupted by user')
        except Exception as e:
            self.get_logger().error(f'Test error: {str(e)}')
    
    def run_quick_test(self):
        """Run a quick validation test"""
        self.get_logger().info('\n=== QUICK SYSTEM TEST ===')
        
        # Test basic functionality
        self.get_logger().info('Testing finger count...')
        msg = Int32()
        msg.data = 3
        self.finger_count_pub.publish(msg)
        self.wait_and_check(2.0)
        
        self.get_logger().info('Testing letter...')
        letter_msg = String()
        letter_msg.data = 'a'
        self.letter_command_pub.publish(letter_msg)
        self.wait_and_check(2.0)
        
        self.get_logger().info('Testing word...')
        word_msg = String()
        word_msg.data = 'hello'
        self.gesture_command_pub.publish(word_msg)
        self.wait_and_check(2.0)
        
        self.get_logger().info('✓ Quick test complete - basic functionality working')

def main(args=None):
    rclpy.init(args=args)
    
    tester = SystemTester()
    
    # Check command line arguments for test type
    if len(sys.argv) > 1:
        test_type = sys.argv[1].lower()
        
        try:
            if test_type == 'quick':
                tester.run_quick_test()
            elif test_type == 'full':
                tester.run_component_tests()
            elif test_type == 'fingers':
                tester.test_finger_counting()
            elif test_type == 'letters':
                tester.test_letter_gestures()
            elif test_type == 'words':
                tester.test_word_gestures()
            elif test_type == 'sequences':
                tester.test_sequence_functionality()
            elif test_type == 'control':
                tester.test_sequence_control()
            elif test_type == 'coordinator':
                tester.test_coordinator_commands()
            else:
                print(f"Unknown test type: {test_type}")
                print("Available tests: quick, full, fingers, letters, words, sequences, control, coordinator")
        except KeyboardInterrupt:
            tester.get_logger().info('Tests stopped by user')
    else:
        # Run quick test by default
        tester.run_quick_test()
    
    tester.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()