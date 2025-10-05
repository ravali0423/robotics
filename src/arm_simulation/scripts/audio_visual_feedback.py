#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool, ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, Vector3
import subprocess
import threading
import time
import os

class AudioVisualFeedback(Node):
    """
    Provides audio feedback for hearing users and visual indicators in RViz
    to show system status (listening, processing, displaying gestures).
    """
    
    def __init__(self):
        super().__init__('audio_visual_feedback')
        
        # Publishers for visual indicators
        self.marker_pub = self.create_publisher(MarkerArray, '/system_status_markers', 10)
        
        # Subscribers for system status
        self.listening_status_sub = self.create_subscription(
            Bool, '/speech_listening_status', self.listening_status_callback, 10)
        
        self.system_status_sub = self.create_subscription(
            String, '/system_status', self.system_status_callback, 10)
        
        self.current_gesture_sub = self.create_subscription(
            String, '/current_gesture', self.current_gesture_callback, 10)
        
        self.sequence_status_sub = self.create_subscription(
            String, '/sequence_status', self.sequence_status_callback, 10)
        
        self.recognized_speech_sub = self.create_subscription(
            String, '/recognized_speech', self.recognized_speech_callback, 10)
        
        # Audio feedback configuration
        self.audio_enabled = True
        self.use_system_sounds = True  # Use system beeps if available
        self.use_text_to_speech = True  # Use TTS for important messages
        
        # Visual feedback configuration
        self.visual_enabled = True
        self.marker_scale = 0.1
        self.marker_position = [0.0, 0.0, 0.3]  # Position relative to hand
        
        # System state
        self.is_listening = False
        self.current_system_mode = "ready"
        self.current_gesture = "neutral"
        self.last_speech_text = ""
        
        # Color definitions for different states
        self.colors = {
            'ready': ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.8),      # Green
            'listening': ColorRGBA(r=0.0, g=0.0, b=1.0, a=0.8),  # Blue
            'processing': ColorRGBA(r=1.0, g=1.0, b=0.0, a=0.8), # Yellow
            'gesturing': ColorRGBA(r=1.0, g=0.5, b=0.0, a=0.8),  # Orange
            'error': ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.8),      # Red
            'neutral': ColorRGBA(r=0.5, g=0.5, b=0.5, a=0.5)     # Gray
        }
        
        # Parameters
        self.declare_parameter('audio_enabled', self.audio_enabled)
        self.declare_parameter('visual_enabled', self.visual_enabled)
        self.declare_parameter('use_system_sounds', self.use_system_sounds)
        self.declare_parameter('use_text_to_speech', self.use_text_to_speech)
        
        # Update from parameters
        self.audio_enabled = self.get_parameter('audio_enabled').value
        self.visual_enabled = self.get_parameter('visual_enabled').value
        self.use_system_sounds = self.get_parameter('use_system_sounds').value
        self.use_text_to_speech = self.get_parameter('use_text_to_speech').value
        
        # Timer for periodic visual updates
        self.visual_timer = self.create_timer(0.5, self.update_visual_indicators)
        
        # Check available audio tools
        self.check_audio_tools()
        
        self.get_logger().info('Audio-Visual Feedback system initialized')
        self.get_logger().info(f'Audio enabled: {self.audio_enabled}')
        self.get_logger().info(f'Visual enabled: {self.visual_enabled}')
        
        # Initial visual setup
        if self.visual_enabled:
            self.update_visual_indicators()
    
    def check_audio_tools(self):
        """Check what audio tools are available on the system"""
        self.available_audio_tools = {}
        
        # Check for common audio tools
        tools_to_check = {
            'espeak': 'espeak',
            'festival': 'festival',
            'paplay': 'paplay',
            'aplay': 'aplay',
            'speaker-test': 'speaker-test'
        }
        
        for tool, command in tools_to_check.items():
            try:
                result = subprocess.run(['which', command], 
                                      capture_output=True, text=True, timeout=2)
                if result.returncode == 0:
                    self.available_audio_tools[tool] = result.stdout.strip()
                    self.get_logger().debug(f'Found audio tool: {tool} at {result.stdout.strip()}')
            except (subprocess.TimeoutExpired, subprocess.SubprocessError):
                pass
        
        if not self.available_audio_tools:
            self.get_logger().warn('No audio tools found. Audio feedback will be limited.')
            self.audio_enabled = False
        else:
            self.get_logger().info(f'Available audio tools: {list(self.available_audio_tools.keys())}')
    
    def listening_status_callback(self, msg):
        """Handle listening status changes"""
        self.is_listening = msg.data
        
        if msg.data:
            self.current_system_mode = "listening"
            self.play_audio_cue('listening_start')
            self.announce("Listening for speech")
        else:
            if self.current_system_mode == "listening":
                self.current_system_mode = "ready"
                self.play_audio_cue('listening_stop')
    
    def system_status_callback(self, msg):
        """Handle system status updates"""
        status = msg.data.lower()
        
        # Extract mode from status string
        if 'mode:' in status:
            mode_part = status.split('mode:')[1].split('|')[0].strip()
            if mode_part != self.current_system_mode:
                self.current_system_mode = mode_part
                self.play_audio_cue(f'mode_{mode_part}')
    
    def current_gesture_callback(self, msg):
        """Handle current gesture updates"""
        self.current_gesture = msg.data
        
        # Announce important gesture changes
        if msg.data == "neutral":
            self.play_audio_cue('gesture_complete')
        elif msg.data.startswith('letter_'):
            letter = msg.data.split('_')[1]
            self.announce(f"Letter {letter.upper()}")
        elif msg.data.startswith('word_'):
            word = msg.data.split('_')[1]
            self.announce(f"Word: {word}")
    
    def sequence_status_callback(self, msg):
        """Handle gesture sequence status"""
        status = msg.data.lower()
        
        if status == 'playing':
            self.current_system_mode = "gesturing"
            self.play_audio_cue('sequence_start')
        elif status == 'completed':
            self.play_audio_cue('sequence_complete')
            self.announce("Gesture sequence completed")
        elif status == 'stopped':
            self.play_audio_cue('sequence_stop')
    
    def recognized_speech_callback(self, msg):
        """Handle recognized speech"""
        self.last_speech_text = msg.data
        self.play_audio_cue('speech_recognized')
        self.announce(f"Recognized: {msg.data}")
    
    def play_audio_cue(self, cue_type: str):
        """Play appropriate audio cue for the given event"""
        if not self.audio_enabled:
            return
        
        # Define audio cues
        cues = {
            'listening_start': {'beep_freq': 800, 'beep_duration': 0.2, 'message': 'Listening'},
            'listening_stop': {'beep_freq': 400, 'beep_duration': 0.1, 'message': None},
            'speech_recognized': {'beep_freq': 1000, 'beep_duration': 0.1, 'message': None},
            'sequence_start': {'beep_freq': 600, 'beep_duration': 0.3, 'message': 'Starting gestures'},
            'sequence_complete': {'beep_freq': 1200, 'beep_duration': 0.5, 'message': 'Gestures complete'},
            'sequence_stop': {'beep_freq': 300, 'beep_duration': 0.2, 'message': 'Gestures stopped'},
            'gesture_complete': {'beep_freq': 500, 'beep_duration': 0.1, 'message': None},
            'error': {'beep_freq': 200, 'beep_duration': 1.0, 'message': 'Error'},
            'mode_ready': {'beep_freq': 900, 'beep_duration': 0.2, 'message': None},
            'mode_processing': {'beep_freq': 700, 'beep_duration': 0.2, 'message': None}
        }
        
        cue = cues.get(cue_type, None)
        if not cue:
            return
        
        # Play in background thread to avoid blocking
        thread = threading.Thread(target=self._play_audio_cue_async, args=(cue,))
        thread.daemon = True
        thread.start()
    
    def _play_audio_cue_async(self, cue: dict):
        """Play audio cue asynchronously"""
        try:
            if self.use_system_sounds and 'speaker-test' in self.available_audio_tools:
                # Generate tone using speaker-test
                freq = cue.get('beep_freq', 800)
                duration = cue.get('beep_duration', 0.2)
                
                # Use speaker-test to generate a tone (if available)
                subprocess.run([
                    'speaker-test', '-t', 'sine', '-f', str(freq), 
                    '-l', '1', '-s', '1'
                ], capture_output=True, timeout=duration + 0.5)
            
            elif 'paplay' in self.available_audio_tools:
                # Use system beep sound if available
                subprocess.run(['paplay', '/usr/share/sounds/alsa/Front_Left.wav'], 
                             capture_output=True, timeout=2.0)
        
        except (subprocess.TimeoutExpired, subprocess.SubprocessError, FileNotFoundError):
            # Fallback to simple print if audio fails
            self.get_logger().debug(f'Audio cue: {cue}')
    
    def announce(self, message: str):
        """Announce message using text-to-speech if available"""
        if not self.audio_enabled or not self.use_text_to_speech:
            return
        
        # Play announcement in background
        thread = threading.Thread(target=self._announce_async, args=(message,))
        thread.daemon = True
        thread.start()
    
    def _announce_async(self, message: str):
        """Announce message asynchronously"""
        try:
            if 'espeak' in self.available_audio_tools:
                subprocess.run(['espeak', '-s', '150', '-v', 'en', message], 
                             capture_output=True, timeout=5.0)
            elif 'festival' in self.available_audio_tools:
                process = subprocess.Popen(['festival', '--tts'], 
                                         stdin=subprocess.PIPE, 
                                         stdout=subprocess.PIPE, 
                                         stderr=subprocess.PIPE)
                process.communicate(input=message.encode(), timeout=5.0)
            else:
                # Fallback to logging the message
                self.get_logger().info(f'TTS: {message}')
                
        except (subprocess.TimeoutExpired, subprocess.SubprocessError):
            self.get_logger().debug(f'TTS failed for: {message}')
    
    def update_visual_indicators(self):
        """Update visual indicators in RViz"""
        if not self.visual_enabled:
            return
        
        marker_array = MarkerArray()
        
        # Main status indicator
        status_marker = self.create_status_marker()
        marker_array.markers.append(status_marker)
        
        # Listening indicator
        if self.is_listening:
            listening_marker = self.create_listening_marker()
            marker_array.markers.append(listening_marker)
        
        # Gesture indicator
        if self.current_gesture != "neutral":
            gesture_marker = self.create_gesture_marker()
            marker_array.markers.append(gesture_marker)
        
        # Text display for last recognized speech
        if self.last_speech_text:
            text_marker = self.create_text_marker()
            marker_array.markers.append(text_marker)
        
        self.marker_pub.publish(marker_array)
    
    def create_status_marker(self) -> Marker:
        """Create main status indicator marker"""
        marker = Marker()
        marker.header.frame_id = "hand_base"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "system_status"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        
        # Position above the hand
        marker.pose.position.x = self.marker_position[0]
        marker.pose.position.y = self.marker_position[1]
        marker.pose.position.z = self.marker_position[2]
        marker.pose.orientation.w = 1.0
        
        # Scale based on system mode
        scale = self.marker_scale
        if self.current_system_mode == "listening":
            scale *= 1.5  # Larger when listening
        
        marker.scale = Vector3(x=scale, y=scale, z=scale)
        
        # Color based on system mode
        marker.color = self.colors.get(self.current_system_mode, self.colors['neutral'])
        
        return marker
    
    def create_listening_marker(self) -> Marker:
        """Create listening indicator (pulsing ring)"""
        marker = Marker()
        marker.header.frame_id = "hand_base"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "listening"
        marker.id = 1
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        
        # Position around the main status marker
        marker.pose.position.x = self.marker_position[0]
        marker.pose.position.y = self.marker_position[1]
        marker.pose.position.z = self.marker_position[2] - 0.02
        marker.pose.orientation.w = 1.0
        
        # Pulsing effect based on time
        pulse = 0.5 + 0.3 * abs(time.time() % 2.0 - 1.0)
        scale = self.marker_scale * 2.0 * pulse
        
        marker.scale = Vector3(x=scale, y=scale, z=0.01)
        marker.color = self.colors['listening']
        marker.color.a = 0.3  # Semi-transparent
        
        return marker
    
    def create_gesture_marker(self) -> Marker:
        """Create gesture indicator marker"""
        marker = Marker()
        marker.header.frame_id = "hand_base"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "gesture"
        marker.id = 2
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        
        # Position next to the hand
        marker.pose.position.x = self.marker_position[0] + 0.15
        marker.pose.position.y = self.marker_position[1]
        marker.pose.position.z = self.marker_position[2]
        marker.pose.orientation.w = 1.0
        
        marker.scale = Vector3(x=0.05, y=0.05, z=0.05)
        marker.color = self.colors['gesturing']
        
        # Display current gesture
        if self.current_gesture.startswith('letter_'):
            marker.text = f"Letter: {self.current_gesture.split('_')[1].upper()}"
        elif self.current_gesture.startswith('finger_count_'):
            count = self.current_gesture.split('_')[2]
            marker.text = f"Count: {count}"
        else:
            marker.text = f"Gesture: {self.current_gesture}"
        
        return marker
    
    def create_text_marker(self) -> Marker:
        """Create text display for recognized speech"""
        marker = Marker()
        marker.header.frame_id = "hand_base"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "speech_text"
        marker.id = 3
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        
        # Position below the hand
        marker.pose.position.x = self.marker_position[0]
        marker.pose.position.y = self.marker_position[1]
        marker.pose.position.z = self.marker_position[2] - 0.15
        marker.pose.orientation.w = 1.0
        
        marker.scale = Vector3(x=0.03, y=0.03, z=0.03)
        marker.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=0.9)  # White
        
        # Truncate long text
        display_text = self.last_speech_text
        if len(display_text) > 30:
            display_text = display_text[:27] + "..."
        
        marker.text = f'"{display_text}"'
        
        return marker

def main(args=None):
    rclpy.init(args=args)
    
    try:
        feedback_node = AudioVisualFeedback()
        rclpy.spin(feedback_node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}')
    finally:
        if 'feedback_node' in locals():
            feedback_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()