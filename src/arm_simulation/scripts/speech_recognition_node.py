#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
import speech_recognition as sr
import threading
import time
import queue

class SpeechRecognitionNode(Node):
    """
    ROS2 node for real-time speech recognition that converts spoken words to text.
    This node serves as the input interface for the speech-to-gesture system.
    """
    
    def __init__(self):
        super().__init__('speech_recognition_node')
        
        # Publishers
        self.text_pub = self.create_publisher(String, '/recognized_speech', 10)
        self.listening_status_pub = self.create_publisher(Bool, '/speech_listening_status', 10)
        
        # Subscribers for control
        self.control_sub = self.create_subscription(
            Bool,
            '/speech_recognition_control',
            self.control_callback,
            10
        )
        
        # Speech recognition setup
        self.recognizer = sr.Recognizer()
        self.microphone = sr.Microphone()
        
        # Configuration
        self.is_listening = False
        self.recognition_language = 'en-US'  # Can be made configurable
        self.energy_threshold = 300  # Adjustable sensitivity
        self.pause_threshold = 0.8  # Seconds of silence before processing
        
        # Threading for non-blocking speech recognition
        self.speech_queue = queue.Queue()
        self.recognition_thread = None
        
        # Parameters
        self.declare_parameter('energy_threshold', self.energy_threshold)
        self.declare_parameter('pause_threshold', self.pause_threshold)
        self.declare_parameter('language', self.recognition_language)
        self.declare_parameter('auto_start', True)
        
        # Update parameters
        self.energy_threshold = self.get_parameter('energy_threshold').value
        self.pause_threshold = self.get_parameter('pause_threshold').value
        self.recognition_language = self.get_parameter('language').value
        auto_start = self.get_parameter('auto_start').value
        
        # Calibrate microphone
        self.calibrate_microphone()
        
        # Auto-start listening if configured
        if auto_start:
            self.start_listening()
        
        self.get_logger().info('Speech Recognition Node initialized')
        self.get_logger().info(f'Language: {self.recognition_language}')
        self.get_logger().info(f'Energy threshold: {self.energy_threshold}')
        self.get_logger().info('Send True to /speech_recognition_control to start/stop listening')
    
    def calibrate_microphone(self):
        """Calibrate the microphone for ambient noise"""
        self.get_logger().info('Calibrating microphone for ambient noise...')
        try:
            with self.microphone as source:
                self.recognizer.adjust_for_ambient_noise(source, duration=1)
            self.recognizer.energy_threshold = self.energy_threshold
            self.recognizer.pause_threshold = self.pause_threshold
            self.get_logger().info('Microphone calibration complete')
        except Exception as e:
            self.get_logger().error(f'Microphone calibration failed: {str(e)}')
    
    def control_callback(self, msg):
        """Handle start/stop commands for speech recognition"""
        if msg.data and not self.is_listening:
            self.start_listening()
        elif not msg.data and self.is_listening:
            self.stop_listening()
    
    def start_listening(self):
        """Start continuous speech recognition"""
        if self.is_listening:
            self.get_logger().warn('Already listening for speech')
            return
        
        self.is_listening = True
        self.publish_listening_status(True)
        
        # Start recognition thread
        self.recognition_thread = threading.Thread(target=self._recognition_worker)
        self.recognition_thread.daemon = True
        self.recognition_thread.start()
        
        self.get_logger().info('Started listening for speech...')
    
    def stop_listening(self):
        """Stop speech recognition"""
        if not self.is_listening:
            self.get_logger().warn('Not currently listening')
            return
        
        self.is_listening = False
        self.publish_listening_status(False)
        
        # Wait for thread to finish
        if self.recognition_thread and self.recognition_thread.is_alive():
            self.recognition_thread.join(timeout=2.0)
        
        self.get_logger().info('Stopped listening for speech')
    
    def _recognition_worker(self):
        """Worker thread for continuous speech recognition"""
        while self.is_listening:
            try:
                with self.microphone as source:
                    # Listen for audio with timeout
                    audio = self.recognizer.listen(source, timeout=1, phrase_time_limit=5)
                
                # Process audio in a separate thread to avoid blocking
                processing_thread = threading.Thread(
                    target=self._process_audio, 
                    args=(audio,)
                )
                processing_thread.daemon = True
                processing_thread.start()
                
            except sr.WaitTimeoutError:
                # Timeout is normal, continue listening
                continue
            except Exception as e:
                self.get_logger().error(f'Error during speech recognition: {str(e)}')
                time.sleep(0.1)
    
    def _process_audio(self, audio):
        """Process audio data and convert to text"""
        try:
            # Try Google Web Speech API (free tier)
            text = self.recognizer.recognize_google(audio, language=self.recognition_language)
            
            if text.strip():
                self.get_logger().info(f'Recognized: "{text}"')
                self.publish_recognized_text(text)
            
        except sr.UnknownValueError:
            self.get_logger().debug('Could not understand audio')
        except sr.RequestError as e:
            self.get_logger().error(f'Could not request results from speech service: {str(e)}')
            # Fallback to offline recognition if available
            try:
                text = self.recognizer.recognize_sphinx(audio)
                if text.strip():
                    self.get_logger().info(f'Recognized (offline): "{text}"')
                    self.publish_recognized_text(text)
            except sr.UnknownValueError:
                self.get_logger().debug('Offline recognition also failed')
            except sr.RequestError as e:
                self.get_logger().error(f'Offline recognition error: {str(e)}')
        except Exception as e:
            self.get_logger().error(f'Unexpected error during audio processing: {str(e)}')
    
    def publish_recognized_text(self, text):
        """Publish recognized text to ROS topic"""
        msg = String()
        msg.data = text.lower().strip()  # Normalize text
        self.text_pub.publish(msg)
    
    def publish_listening_status(self, status):
        """Publish current listening status"""
        msg = Bool()
        msg.data = status
        self.listening_status_pub.publish(msg)
    
    def destroy_node(self):
        """Clean shutdown"""
        self.stop_listening()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    try:
        speech_node = SpeechRecognitionNode()
        rclpy.spin(speech_node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}')
    finally:
        if 'speech_node' in locals():
            speech_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()