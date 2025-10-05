#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool, Int32
from geometry_msgs.msg import PoseArray
import time
import threading
from typing import List, Tuple, Dict

class GestureSequencer(Node):
    """
    Manages sequences of hand gestures for spelling words and complex signs.
    Handles timing, transitions, and coordination between multiple gestures.
    """
    
    def __init__(self):
        super().__init__('gesture_sequencer')
        
        # Publishers
        self.gesture_command_pub = self.create_publisher(String, '/gesture_command', 10)
        self.letter_command_pub = self.create_publisher(String, '/letter_command', 10)
        self.sequence_status_pub = self.create_publisher(String, '/sequence_status', 10)
        self.sequence_progress_pub = self.create_publisher(Int32, '/sequence_progress', 10)
        
        # Subscribers
        self.text_sequence_sub = self.create_subscription(
            String, '/text_to_sequence', self.text_sequence_callback, 10)
        
        self.control_sub = self.create_subscription(
            String, '/sequence_control', self.control_callback, 10)
        
        # Sequence management
        self.current_sequence = []
        self.sequence_index = 0
        self.is_playing = False
        self.is_paused = False
        self.sequence_thread = None
        self.sequence_name = ""
        
        # Timing parameters
        self.letter_duration = 1.5  # seconds per letter
        self.word_pause = 2.0      # pause between words
        self.letter_pause = 0.8    # pause between letters
        self.gesture_duration = 2.5 # duration for complete word gestures
        
        # Parameters
        self.declare_parameter('letter_duration', self.letter_duration)
        self.declare_parameter('word_pause', self.word_pause)
        self.declare_parameter('letter_pause', self.letter_pause)
        self.declare_parameter('gesture_duration', self.gesture_duration)
        
        # Update from parameters
        self.letter_duration = self.get_parameter('letter_duration').value
        self.word_pause = self.get_parameter('word_pause').value
        self.letter_pause = self.get_parameter('letter_pause').value
        self.gesture_duration = self.get_parameter('gesture_duration').value
        
        self.get_logger().info('Gesture Sequencer initialized')
        self.get_logger().info('Commands:')
        self.get_logger().info('  - /text_to_sequence (String): Text to convert to gesture sequence')
        self.get_logger().info('  - /sequence_control (String): "play", "pause", "stop", "reset"')
        self.get_logger().info(f'Timing: letter={self.letter_duration}s, word_pause={self.word_pause}s')
    
    def text_sequence_callback(self, msg):
        """Convert text to gesture sequence"""
        text = msg.data.strip()
        if not text:
            self.get_logger().warn('Empty text received')
            return
        
        self.get_logger().info(f'Creating sequence for: "{text}"')
        self.create_sequence_from_text(text)
    
    def control_callback(self, msg):
        """Handle sequence control commands"""
        command = msg.data.lower().strip()
        
        if command == 'play':
            self.play_sequence()
        elif command == 'pause':
            self.pause_sequence()
        elif command == 'stop':
            self.stop_sequence()
        elif command == 'reset':
            self.reset_sequence()
        elif command == 'next':
            self.next_gesture()
        elif command == 'previous':
            self.previous_gesture()
        else:
            self.get_logger().warn(f'Unknown control command: "{command}"')
            self.get_logger().info('Available commands: play, pause, stop, reset, next, previous')
    
    def create_sequence_from_text(self, text: str):
        """
        Create a gesture sequence from input text
        
        Args:
            text: Input text to convert to gestures
        """
        # Stop any current sequence
        self.stop_sequence()
        
        # Simple sequence creation - just spell out characters
        self.current_sequence = []
        self.sequence_name = text
        
        # Common words that have special gestures
        common_words = {'hello', 'goodbye', 'please', 'thank you', 'yes', 'no'}
        
        words = text.lower().split()
        
        for word_index, word in enumerate(words):
            if word in common_words:
                # Use word gesture
                self.current_sequence.append({
                    'type': 'gesture',
                    'command': word,
                    'duration': self.gesture_duration,
                    'description': f'Word: {word}',
                    'config': {}
                })
            else:
                # Finger spell the word
                for char_index, char in enumerate(word):
                    if char.isalpha():
                        self.current_sequence.append({
                            'type': 'letter',
                            'command': char,
                            'duration': self.letter_duration,
                            'description': f'Letter: {char}',
                            'config': {}
                        })
                        
                        # Add pause between letters (except last letter)
                        if char_index < len(word) - 1:
                            self.current_sequence.append({
                                'type': 'pause',
                                'command': 'neutral',
                                'duration': self.letter_pause,
                                'description': 'Letter pause',
                                'config': {}
                            })
            
            # Add pause between words (except last word)
            if word_index < len(words) - 1:
                self.current_sequence.append({
                    'type': 'pause',
                    'command': 'neutral',
                    'duration': self.word_pause,
                    'description': 'Word pause',
                    'config': {}
                })
        
        self.sequence_index = 0
        self.get_logger().info(f'Created sequence with {len(self.current_sequence)} gestures')
        self.publish_sequence_status(f'Sequence created: "{text}" ({len(self.current_sequence)} gestures)')
        
        # Automatically start playing
        self.play_sequence()
    
    def play_sequence(self):
        """Start or resume playing the current sequence"""
        if not self.current_sequence:
            self.get_logger().warn('No sequence to play')
            return
        
        if self.is_playing:
            self.get_logger().info('Sequence already playing')
            return
        
        self.is_playing = True
        self.is_paused = False
        
        # Start sequence playback thread
        self.sequence_thread = threading.Thread(target=self._sequence_worker)
        self.sequence_thread.daemon = True
        self.sequence_thread.start()
        
        self.get_logger().info(f'Playing sequence: "{self.sequence_name}"')
        self.publish_sequence_status('Playing')
    
    def pause_sequence(self):
        """Pause the current sequence"""
        if not self.is_playing:
            self.get_logger().warn('No sequence currently playing')
            return
        
        self.is_paused = True
        self.get_logger().info('Sequence paused')
        self.publish_sequence_status('Paused')
    
    def stop_sequence(self):
        """Stop the current sequence"""
        self.is_playing = False
        self.is_paused = False
        
        if self.sequence_thread and self.sequence_thread.is_alive():
            self.sequence_thread.join(timeout=1.0)
        
        # Reset to neutral position
        self._send_gesture_command('gesture', 'neutral')
        
        self.get_logger().info('Sequence stopped')
        self.publish_sequence_status('Stopped')
    
    def reset_sequence(self):
        """Reset sequence to beginning"""
        self.stop_sequence()
        self.sequence_index = 0
        self.get_logger().info('Sequence reset to beginning')
        self.publish_sequence_status('Reset')
        self.publish_sequence_progress(0)
    
    def next_gesture(self):
        """Skip to next gesture in sequence"""
        if not self.current_sequence:
            return
        
        self.sequence_index = min(self.sequence_index + 1, len(self.current_sequence) - 1)
        self.get_logger().info(f'Skipped to gesture {self.sequence_index + 1}/{len(self.current_sequence)}')
        self.publish_sequence_progress(self.sequence_index)
    
    def previous_gesture(self):
        """Go to previous gesture in sequence"""
        if not self.current_sequence:
            return
        
        self.sequence_index = max(self.sequence_index - 1, 0)
        self.get_logger().info(f'Moved to gesture {self.sequence_index + 1}/{len(self.current_sequence)}')
        self.publish_sequence_progress(self.sequence_index)
    
    def _sequence_worker(self):
        """Worker thread for sequence playback"""
        while self.is_playing and self.sequence_index < len(self.current_sequence):
            # Handle pause
            while self.is_paused and self.is_playing:
                time.sleep(0.1)
            
            if not self.is_playing:
                break
            
            # Get current gesture
            gesture = self.current_sequence[self.sequence_index]
            
            # Execute gesture
            self.get_logger().info(f'Gesture {self.sequence_index + 1}/{len(self.current_sequence)}: {gesture["description"]}')
            self._send_gesture_command(gesture['type'], gesture['command'])
            
            # Publish progress
            self.publish_sequence_progress(self.sequence_index)
            
            # Wait for duration
            start_time = time.time()
            while time.time() - start_time < gesture['duration'] and self.is_playing:
                if self.is_paused:
                    break
                time.sleep(0.1)
            
            # Move to next gesture
            self.sequence_index += 1
        
        # Sequence completed or stopped
        if self.is_playing and self.sequence_index >= len(self.current_sequence):
            self.get_logger().info('Sequence completed')
            self.publish_sequence_status('Completed')
            self.is_playing = False
            # Return to neutral
            self._send_gesture_command('gesture', 'neutral')
    
    def _send_gesture_command(self, gesture_type: str, command: str):
        """Send gesture command to appropriate topic"""
        msg = String()
        msg.data = command
        
        if gesture_type == 'letter':
            self.letter_command_pub.publish(msg)
        else:  # gesture or pause
            self.gesture_command_pub.publish(msg)
    
    def publish_sequence_status(self, status: str):
        """Publish sequence status"""
        msg = String()
        msg.data = status
        self.sequence_status_pub.publish(msg)
    
    def publish_sequence_progress(self, index: int):
        """Publish current sequence progress"""
        msg = Int32()
        msg.data = index
        self.sequence_progress_pub.publish(msg)
    
    def get_sequence_info(self):
        """Get information about current sequence"""
        if not self.current_sequence:
            return {'status': 'No sequence loaded'}
        
        return {
            'name': self.sequence_name,
            'total_gestures': len(self.current_sequence),
            'current_index': self.sequence_index,
            'is_playing': self.is_playing,
            'is_paused': self.is_paused,
            'current_gesture': self.current_sequence[self.sequence_index] if self.sequence_index < len(self.current_sequence) else None
        }
    
    def create_demo_sequences(self):
        """Create some demo sequences for testing"""
        demo_texts = [
            "hello",
            "abc",
            "123",
            "hello world",
            "thank you"
        ]
        
        self.get_logger().info('Demo sequences available:')
        for text in demo_texts:
            self.get_logger().info(f'  - "{text}"')
    
    def destroy_node(self):
        """Clean shutdown"""
        self.stop_sequence()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    try:
        sequencer = GestureSequencer()
        
        # Show demo sequences
        sequencer.create_demo_sequences()
        
        rclpy.spin(sequencer)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}')
    finally:
        if 'sequencer' in locals():
            sequencer.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()