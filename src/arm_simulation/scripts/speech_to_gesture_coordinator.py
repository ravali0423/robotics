#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool, Int32
import time
import re
from typing import Dict, List, Optional
import sys
import os

# Add the scripts directory to the Python path
script_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(script_dir)

class SpeechToGestureCoordinator(Node):
    """
    Main coordinator node that orchestrates the complete speech-to-gesture pipeline.
    Manages the flow: Speech Recognition → Text Processing → Sign Language Mapping → Gesture Execution
    """
    
    def __init__(self):
        super().__init__('speech_to_gesture_coordinator')
        
        # Publishers for controlling other nodes
        self.speech_control_pub = self.create_publisher(Bool, '/speech_recognition_control', 10)
        self.text_sequence_pub = self.create_publisher(String, '/text_to_sequence', 10)
        self.sequence_control_pub = self.create_publisher(String, '/sequence_control', 10)
        self.system_status_pub = self.create_publisher(String, '/system_status', 10)
        
        # Direct command publishers for enhanced hand controller
        self.letter_command_pub = self.create_publisher(String, '/letter_command', 10)
        self.gesture_command_pub = self.create_publisher(String, '/gesture_command', 10)
        
        # Subscribers for receiving data from other nodes
        self.speech_text_sub = self.create_subscription(
            String, '/recognized_speech', self.speech_text_callback, 10)
        
        self.listening_status_sub = self.create_subscription(
            Bool, '/speech_listening_status', self.listening_status_callback, 10)
        
        self.sequence_status_sub = self.create_subscription(
            String, '/sequence_status', self.sequence_status_callback, 10)
        
        self.current_gesture_sub = self.create_subscription(
            String, '/current_gesture', self.current_gesture_callback, 10)
        
        # User control subscriber
        self.user_control_sub = self.create_subscription(
            String, '/user_command', self.user_control_callback, 10)
        
        # System state
        self.is_listening = False
        self.is_processing = False
        self.is_gesture_active = False
        self.current_gesture = "neutral"
        self.last_recognized_text = ""
        self.system_mode = "ready"  # ready, listening, processing, gesturing, error
        
        # Configuration
        self.auto_start_listening = True
        self.gesture_timeout = 30.0  # seconds before returning to ready state
        self.min_text_length = 1  # minimum characters to process (allow single letters)
        self.max_text_length = 200  # maximum characters to process
        
        # Text processing filters
        self.common_false_positives = [
            'uh', 'um', 'ah', 'er', 'uh-huh', 'mm-hmm', 'ok', 'okay'
        ]
        
        # Parameters
        self.declare_parameter('auto_start_listening', self.auto_start_listening)
        self.declare_parameter('gesture_timeout', self.gesture_timeout)
        self.declare_parameter('min_text_length', self.min_text_length)
        self.declare_parameter('max_text_length', self.max_text_length)
        
        # Update from parameters
        self.auto_start_listening = self.get_parameter('auto_start_listening').value
        self.gesture_timeout = self.get_parameter('gesture_timeout').value
        self.min_text_length = self.get_parameter('min_text_length').value
        self.max_text_length = self.get_parameter('max_text_length').value
        
        # Timing
        self.last_gesture_time = time.time()
        self.status_timer = self.create_timer(1.0, self.check_system_status)
        
        self.get_logger().info('Speech-to-Gesture Coordinator initialized')
        self.get_logger().info('=== ASSISTIVE TECHNOLOGY FOR DEAF COMMUNITY ===')
        self.get_logger().info('This system translates spoken words into sign language gestures')
        self.get_logger().info('')
        self.get_logger().info('User Commands (publish to /user_command):')
        self.get_logger().info('  - "start listening" - Begin speech recognition')
        self.get_logger().info('  - "stop listening" - Stop speech recognition')
        self.get_logger().info('  - "gesture [text]" - Manually trigger gesture for text')
        self.get_logger().info('  - "demo alphabet" - Demonstrate ASL alphabet')
        self.get_logger().info('  - "demo numbers" - Demonstrate numbers 0-9')
        self.get_logger().info('  - "demo words" - Demonstrate common words')
        self.get_logger().info('  - "status" - Show system status')
        self.get_logger().info('  - "reset" - Reset system to ready state')
        self.get_logger().info('')
        
        # Auto-start if configured
        if self.auto_start_listening:
            self.get_logger().info('Auto-starting speech recognition...')
            self.start_listening()
        else:
            self.set_system_mode('ready')
            self.get_logger().info('System ready. Send "start listening" to begin.')
    
    def speech_text_callback(self, msg):
        """Handle recognized speech text"""
        text = msg.data.strip().lower()
        
        if not text:
            return
        
        self.last_recognized_text = text
        self.get_logger().info(f'Received speech: "{text}"')
        
        # Process the text
        self.process_recognized_text(text)
    
    def listening_status_callback(self, msg):
        """Handle speech recognition listening status"""
        self.is_listening = msg.data
        
        if self.is_listening:
            self.set_system_mode('listening')
        elif self.system_mode == 'listening':
            self.set_system_mode('ready')
    
    def sequence_status_callback(self, msg):
        """Handle gesture sequence status"""
        status = msg.data.lower()
        self.get_logger().info(f'Received sequence status: "{status}"')
        
        if status == 'playing':
            self.is_gesture_active = True
            self.set_system_mode('gesturing')
            self.last_gesture_time = time.time()
        elif status in ['completed', 'stopped']:
            self.get_logger().info(f'Sequence {status} - is_gesture_active={self.is_gesture_active}, system_mode={self.system_mode}')
            self.is_gesture_active = False
            if self.system_mode in ['gesturing', 'processing']:  # Handle both gesturing and processing modes
                self.set_system_mode('ready')
                # Restart listening if auto mode
                if self.auto_start_listening:
                    self.get_logger().info('Auto-restart enabled - restarting speech recognition in 1 second')
                    time.sleep(1.0)  # Brief pause
                    self.start_listening()
                else:
                    self.get_logger().info('Auto-restart disabled - speech recognition NOT restarted')
            else:
                self.get_logger().info(f'System mode is {self.system_mode}, not restarting speech recognition')
    
    def current_gesture_callback(self, msg):
        """Handle current gesture updates"""
        self.current_gesture = msg.data
    
    def user_control_callback(self, msg):
        """Handle user control commands"""
        command = msg.data.strip().lower()
        
        self.get_logger().info(f'User command: "{command}"')
        
        if command == 'start listening':
            self.start_listening()
        elif command == 'stop listening':
            self.stop_listening()
        elif command.startswith('gesture '):
            text = command[8:]  # Remove 'gesture ' prefix
            self.process_recognized_text(text)
        elif command == 'demo alphabet':
            self.demo_alphabet()
        elif command == 'demo numbers':
            self.demo_numbers()
        elif command == 'demo words':
            self.demo_words()
        elif command == 'status':
            self.show_status()
        elif command == 'reset':
            self.reset_system()
        elif command == 'stop gesture':
            self.stop_current_gesture()
        else:
            self.get_logger().warn(f'Unknown command: "{command}"')
            self.show_available_commands()
    
    def process_recognized_text(self, text: str):
        """Process recognized text and convert to gestures"""
        # Validate text length
        if len(text) < self.min_text_length:
            self.get_logger().debug(f'Text too short: "{text}" (min: {self.min_text_length})')
            return
        
        if len(text) > self.max_text_length:
            self.get_logger().warn(f'Text too long: {len(text)} chars (max: {self.max_text_length})')
            text = text[:self.max_text_length]
        
        # Filter out common false positives
        if text in self.common_false_positives:
            self.get_logger().debug(f'Filtered false positive: "{text}"')
            return
        
        # Clean and normalize text
        cleaned_text = self.clean_text(text)
        if not cleaned_text:
            return
        
        self.get_logger().info(f'Processing text for gestures: "{cleaned_text}"')
        self.set_system_mode('processing')
        
        # Stop current listening
        self.stop_listening()
        
        # Check if it's a single letter command
        if len(cleaned_text) == 1 and cleaned_text.isalpha():
            self.get_logger().info(f'Sending single letter: "{cleaned_text}"')
            msg = String()
            msg.data = cleaned_text
            self.letter_command_pub.publish(msg)
            
            # Use threading timer for delayed restart
            import threading
            threading.Timer(3.0, self.restart_listening_after_letter).start()
            
        # Check for common gesture words
        elif cleaned_text in ['hello', 'hi', 'goodbye', 'bye', 'thank you', 'thanks', 'please']:
            self.get_logger().info(f'Sending gesture command: "{cleaned_text}"')
            msg = String()
            msg.data = cleaned_text
            self.gesture_command_pub.publish(msg)
            
            # Use threading timer for delayed restart
            import threading
            threading.Timer(4.0, self.restart_listening_after_gesture).start()
            
        else:
            # Send to gesture sequencer for word/phrase spelling
            msg = String()
            msg.data = cleaned_text
            self.text_sequence_pub.publish(msg)
            self.get_logger().info(f'Sent to gesture system: "{cleaned_text}"')
    
    def clean_text(self, text: str) -> str:
        """Clean and normalize text for gesture processing"""
        # Convert to lowercase
        text = text.lower().strip()
        
        # Remove extra whitespace
        text = re.sub(r'\s+', ' ', text)
        
        # Remove common filler words at the beginning
        fillers = ['um', 'uh', 'er', 'ah', 'like', 'you know']
        words = text.split()
        while words and words[0] in fillers:
            words.pop(0)
        
        # Reconstruct text
        text = ' '.join(words)
        
        # Handle common substitutions
        substitutions = {
            'okay': 'ok',
            'alright': 'ok',
            'hey': 'hello',
            'hi': 'hello',
            'thanks': 'thank you',
            'thx': 'thank you',
            'bye': 'goodbye',
            'see you': 'goodbye'
        }
        
        for old, new in substitutions.items():
            text = text.replace(old, new)
        
        return text
    
    def start_listening(self):
        """Start speech recognition"""
        if self.is_listening:
            self.get_logger().info('Already listening')
            return
        
        self.get_logger().info('Starting speech recognition...')
        msg = Bool()
        msg.data = True
        self.speech_control_pub.publish(msg)
        
        self.set_system_mode('listening')
    
    def stop_listening(self):
        """Stop speech recognition"""
        if not self.is_listening:
            return
        
        self.get_logger().info('Stopping speech recognition')
        msg = Bool()
        msg.data = False
        self.speech_control_pub.publish(msg)
    
    def stop_current_gesture(self):
        """Stop current gesture sequence"""
        self.get_logger().info('Stopping current gesture')
        msg = String()
        msg.data = 'stop'
        self.sequence_control_pub.publish(msg)
    
    def demo_alphabet(self):
        """Demonstrate ASL alphabet"""
        self.get_logger().info('Starting alphabet demonstration')
        self.stop_listening()
        
        alphabet = 'abcdefghijklmnopqrstuvwxyz'
        msg = String()
        msg.data = alphabet
        self.text_sequence_pub.publish(msg)
    
    def demo_numbers(self):
        """Demonstrate numbers 0-9"""
        self.get_logger().info('Starting numbers demonstration')
        self.stop_listening()
        
        numbers = '0123456789'
        msg = String()
        msg.data = numbers
        self.text_sequence_pub.publish(msg)
    
    def demo_words(self):
        """Demonstrate common words"""
        self.get_logger().info('Starting words demonstration')
        self.stop_listening()
        
        words = 'hello goodbye please thank you yes no'
        msg = String()
        msg.data = words
        self.text_sequence_pub.publish(msg)
    
    def reset_system(self):
        """Reset system to ready state"""
        self.get_logger().info('Resetting system...')
        
        # Stop everything
        self.stop_listening()
        self.stop_current_gesture()
        
        # Reset state
        self.is_processing = False
        self.is_gesture_active = False
        self.current_gesture = "neutral"
        self.last_recognized_text = ""
        
        # Return to neutral position
        msg = String()
        msg.data = 'neutral'
        self.text_sequence_pub.publish(msg)
        
        self.set_system_mode('ready')
        self.get_logger().info('System reset complete')
    
    def set_system_mode(self, mode: str):
        """Set system mode and publish status"""
        self.system_mode = mode
        self.publish_system_status(f'Mode: {mode}')
    
    def publish_system_status(self, status: str):
        """Publish system status"""
        msg = String()
        msg.data = status
        self.system_status_pub.publish(msg)
    
    def check_system_status(self):
        """Periodic system status check"""
        current_time = time.time()
        
        # Check for gesture timeout
        if self.is_gesture_active and (current_time - self.last_gesture_time) > self.gesture_timeout:
            self.get_logger().warn('Gesture timeout - resetting to ready state')
            self.reset_system()
        
        # Publish periodic status
        status_info = {
            'mode': self.system_mode,
            'listening': self.is_listening,
            'gesture_active': self.is_gesture_active,
            'current_gesture': self.current_gesture,
            'last_text': self.last_recognized_text[-50:] if self.last_recognized_text else 'None'
        }
        
        status_str = f"System: {status_info['mode']} | Listening: {status_info['listening']} | Gesture: {status_info['current_gesture']}"
        self.publish_system_status(status_str)
    
    def show_status(self):
        """Display detailed system status"""
        self.get_logger().info('=== SYSTEM STATUS ===')
        self.get_logger().info(f'Mode: {self.system_mode}')
        self.get_logger().info(f'Listening: {self.is_listening}')
        self.get_logger().info(f'Processing: {self.is_processing}')
        self.get_logger().info(f'Gesture Active: {self.is_gesture_active}')
        self.get_logger().info(f'Current Gesture: {self.current_gesture}')
        self.get_logger().info(f'Last Text: "{self.last_recognized_text}"')
        self.get_logger().info('====================')
    
    def restart_listening_after_letter(self):
        """Restart listening after a single letter gesture completes"""
        self.get_logger().info('Letter gesture completed, restarting speech recognition...')
        self.set_system_mode('ready')
        self.start_listening()
    
    def restart_listening_after_gesture(self):
        """Restart listening after a gesture command completes"""
        self.get_logger().info('Gesture command completed, restarting speech recognition...')
        self.set_system_mode('ready')
        self.start_listening()
    
    def show_available_commands(self):
        """Show available user commands"""
        self.get_logger().info('Available commands:')
        commands = [
            'start listening', 'stop listening', 'gesture [text]',
            'demo alphabet', 'demo numbers', 'demo words',
            'status', 'reset', 'stop gesture'
        ]
        for cmd in commands:
            self.get_logger().info(f'  - {cmd}')

def main(args=None):
    rclpy.init(args=args)
    
    try:
        coordinator = SpeechToGestureCoordinator()
        
        # Display startup message
        print("\n" + "="*60)
        print("    SPEECH-TO-GESTURE SYSTEM FOR DEAF COMMUNITY")
        print("="*60)
        print("This system translates spoken words into sign language gestures")
        print("using a robotic hand to help bridge communication gaps.")
        print("")
        print("To control the system, publish commands to /user_command:")
        print("  ros2 topic pub /user_command std_msgs/msg/String '{data: \"start listening\"}'")
        print("="*60 + "\n")
        
        rclpy.spin(coordinator)
        
    except KeyboardInterrupt:
        print("\nShutting down Speech-to-Gesture System...")
    except Exception as e:
        print(f'Error: {e}')
    finally:
        if 'coordinator' in locals():
            coordinator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()