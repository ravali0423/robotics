#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32
import time
import sys

class SpeechToGestureDemo(Node):
    """
    Demo script that showcases the speech-to-gesture system capabilities.
    Demonstrates alphabet, numbers, common words, and interactive mode.
    """
    
    def __init__(self):
        super().__init__('speech_to_gesture_demo')
        
        # Publishers for controlling the system
        self.text_sequence_pub = self.create_publisher(String, '/text_to_sequence', 10)
        self.gesture_command_pub = self.create_publisher(String, '/gesture_command', 10)
        self.letter_command_pub = self.create_publisher(String, '/letter_command', 10)
        self.user_command_pub = self.create_publisher(String, '/user_command', 10)
        
        # Wait for system to be ready
        time.sleep(2)
        
        self.get_logger().info('=== SPEECH-TO-GESTURE SYSTEM DEMO ===')
        self.get_logger().info('This demo showcases assistive technology for the deaf community')
        self.get_logger().info('===============================================')
    
    def send_text_sequence(self, text: str, description: str = ""):
        """Send text to be converted to gesture sequence"""
        if description:
            self.get_logger().info(f'{description}: "{text}"')
        else:
            self.get_logger().info(f'Sending text: "{text}"')
        
        msg = String()
        msg.data = text
        self.text_sequence_pub.publish(msg)
    
    def send_user_command(self, command: str):
        """Send user command"""
        self.get_logger().info(f'Command: {command}')
        msg = String()
        msg.data = command
        self.user_command_pub.publish(msg)
    
    def wait_for_completion(self, duration: float):
        """Wait for gesture completion"""
        self.get_logger().info(f'Waiting {duration} seconds for completion...')
        time.sleep(duration)
    
    def demo_alphabet(self):
        """Demonstrate ASL alphabet"""
        self.get_logger().info('\n--- ASL ALPHABET DEMONSTRATION ---')
        self.send_user_command('demo alphabet')
        self.wait_for_completion(30)  # Allow time for full alphabet
    
    def demo_numbers(self):
        """Demonstrate numbers 0-9"""
        self.get_logger().info('\n--- NUMBERS DEMONSTRATION ---')
        self.send_user_command('demo numbers')
        self.wait_for_completion(15)  # Allow time for numbers
    
    def demo_common_words(self):
        """Demonstrate common ASL words"""
        self.get_logger().info('\n--- COMMON WORDS DEMONSTRATION ---')
        
        common_words = [
            'hello',
            'goodbye', 
            'please',
            'thank you',
            'yes',
            'no'
        ]
        
        for word in common_words:
            self.send_text_sequence(word, f"Common word")
            self.wait_for_completion(3)
    
    def demo_finger_spelling(self):
        """Demonstrate finger spelling with simple words"""
        self.get_logger().info('\n--- FINGER SPELLING DEMONSTRATION ---')
        
        words_to_spell = [
            'cat',
            'dog', 
            'help',
            'love'
        ]
        
        for word in words_to_spell:
            self.send_text_sequence(word, f"Finger spelling")
            self.wait_for_completion(len(word) * 2 + 2)  # 2 seconds per letter + buffer
    
    def demo_sentences(self):
        """Demonstrate simple sentences"""
        self.get_logger().info('\n--- SENTENCE DEMONSTRATION ---')
        
        sentences = [
            'hello world',
            'thank you very much',
            'how are you',
            'nice to meet you'
        ]
        
        for sentence in sentences:
            self.send_text_sequence(sentence, f"Sentence")
            # Calculate time based on sentence length
            wait_time = len(sentence.split()) * 3 + len(sentence) * 0.5
            self.wait_for_completion(wait_time)
    
    def demo_emergency_phrases(self):
        """Demonstrate emergency/important phrases"""
        self.get_logger().info('\n--- EMERGENCY PHRASES DEMONSTRATION ---')
        
        emergency_phrases = [
            'help',
            'emergency',
            'call 911',
            'doctor',
            'hospital'
        ]
        
        for phrase in emergency_phrases:
            self.send_text_sequence(phrase, f"Emergency phrase")
            wait_time = len(phrase) * 1.5 + 2
            self.wait_for_completion(wait_time)
    
    def interactive_demo(self):
        """Interactive demonstration allowing manual input"""
        self.get_logger().info('\n--- INTERACTIVE DEMONSTRATION ---')
        self.get_logger().info('You can now control the system using these commands:')
        self.get_logger().info('  ros2 topic pub /text_to_sequence std_msgs/msg/String \'{data: "your text here"}\'')
        self.get_logger().info('  ros2 topic pub /user_command std_msgs/msg/String \'{data: "start listening"}\'')
        self.get_logger().info('Press Ctrl+C to exit interactive mode')
        
        try:
            while rclpy.ok():
                time.sleep(1)
        except KeyboardInterrupt:
            self.get_logger().info('Exiting interactive mode')
    
    def run_full_demo(self):
        """Run the complete demonstration"""
        self.get_logger().info('Starting full demonstration in 3 seconds...')
        time.sleep(3)
        
        try:
            # Run demonstrations in sequence
            self.demo_alphabet()
            time.sleep(2)
            
            self.demo_numbers()
            time.sleep(2)
            
            self.demo_common_words()
            time.sleep(2)
            
            self.demo_finger_spelling()
            time.sleep(2)
            
            self.demo_sentences()
            time.sleep(2)
            
            self.demo_emergency_phrases()
            time.sleep(2)
            
            self.get_logger().info('\n=== DEMONSTRATION COMPLETE ===')
            self.get_logger().info('The system is now ready for interactive use.')
            
            # Switch to interactive mode
            self.interactive_demo()
            
        except KeyboardInterrupt:
            self.get_logger().info('\nDemo interrupted by user')
        except Exception as e:
            self.get_logger().error(f'Demo error: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    
    demo = SpeechToGestureDemo()
    
    # Check command line arguments for demo type
    if len(sys.argv) > 1:
        demo_type = sys.argv[1].lower()
        
        try:
            if demo_type == 'alphabet':
                demo.demo_alphabet()
            elif demo_type == 'numbers':
                demo.demo_numbers()
            elif demo_type == 'words':
                demo.demo_common_words()
            elif demo_type == 'spelling':
                demo.demo_finger_spelling()
            elif demo_type == 'sentences':
                demo.demo_sentences()
            elif demo_type == 'emergency':
                demo.demo_emergency_phrases()
            elif demo_type == 'interactive':
                demo.interactive_demo()
            elif demo_type == 'full':
                demo.run_full_demo()
            else:
                print(f"Unknown demo type: {demo_type}")
                print("Available demos: alphabet, numbers, words, spelling, sentences, emergency, interactive, full")
        except KeyboardInterrupt:
            demo.get_logger().info('Demo stopped by user')
    else:
        # Run full demo by default
        demo.run_full_demo()
    
    demo.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()