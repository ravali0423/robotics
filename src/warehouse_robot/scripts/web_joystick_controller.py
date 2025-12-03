#!/usr/bin/env python3
"""
Web-based Joystick Controller for Warehouse Robot
Provides a web interface with virtual joystick control
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import threading
import http.server
import socketserver
import json
import urllib.parse
import webbrowser
import os

class WebJoystickController(Node):
    def __init__(self):
        super().__init__('web_joystick_controller')
        
        # Publisher for robot velocity commands
        self.cmd_vel_publisher = self.create_publisher(
            Twist, 
            '/model/warehouse_car/cmd_vel', 
            10
        )
        
        # Robot parameters
        self.max_linear_vel = 1.0
        self.max_angular_vel = 2.0
        self.current_linear = 0.0
        self.current_angular = 0.0
        
        self.get_logger().info('🌐 Web Joystick Controller Started!')
        
    def publish_velocity(self, linear_x=0.0, angular_z=0.0):
        """Publish velocity command to robot"""
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        self.cmd_vel_publisher.publish(msg)
        
        # Update current velocities
        self.current_linear = linear_x
        self.current_angular = angular_z
        
        # Log significant movements
        if abs(linear_x) > 0.01 or abs(angular_z) > 0.01:
            self.get_logger().info(f'🚗 Velocity: linear={linear_x:.2f}, angular={angular_z:.2f}')

class JoystickHTTPHandler(http.server.SimpleHTTPRequestHandler):
    def __init__(self, *args, robot_controller=None, **kwargs):
        self.robot_controller = robot_controller
        super().__init__(*args, **kwargs)
        
    def do_POST(self):
        """Handle POST requests for joystick commands"""
        if self.path == '/cmd_vel':
            content_length = int(self.headers['Content-Length'])
            post_data = self.rfile.read(content_length)
            
            try:
                data = json.loads(post_data.decode('utf-8'))
                linear_x = float(data.get('linear_x', 0.0))
                angular_z = float(data.get('angular_z', 0.0))
                
                # Publish the velocity command
                if self.robot_controller:
                    self.robot_controller.publish_velocity(linear_x, angular_z)
                
                # Send response
                self.send_response(200)
                self.send_header('Content-type', 'application/json')
                self.send_header('Access-Control-Allow-Origin', '*')
                self.end_headers()
                self.wfile.write(json.dumps({'status': 'success'}).encode('utf-8'))
                
            except Exception as e:
                self.send_response(400)
                self.send_header('Content-type', 'application/json')
                self.send_header('Access-Control-Allow-Origin', '*')
                self.end_headers()
                self.wfile.write(json.dumps({'error': str(e)}).encode('utf-8'))
        else:
            self.send_response(404)
            self.end_headers()
            
    def do_OPTIONS(self):
        """Handle CORS preflight requests"""
        self.send_response(200)
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Access-Control-Allow-Methods', 'POST, OPTIONS')
        self.send_header('Access-Control-Allow-Headers', 'Content-Type')
        self.end_headers()

def create_joystick_html():
    """Create the HTML file for the joystick interface"""
    html_content = """<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>🚗 Warehouse Robot Joystick Controller</title>
    <style>
        body {
            font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
            background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
            margin: 0;
            padding: 20px;
            min-height: 100vh;
            display: flex;
            justify-content: center;
            align-items: center;
            color: white;
        }
        
        .controller {
            background: rgba(255, 255, 255, 0.1);
            backdrop-filter: blur(10px);
            border-radius: 20px;
            padding: 30px;
            text-align: center;
            box-shadow: 0 8px 32px rgba(0, 0, 0, 0.3);
            border: 1px solid rgba(255, 255, 255, 0.2);
            max-width: 500px;
            width: 100%;
        }
        
        .title {
            font-size: 2em;
            margin-bottom: 10px;
            text-shadow: 2px 2px 4px rgba(0, 0, 0, 0.3);
        }
        
        .subtitle {
            font-size: 1.2em;
            margin-bottom: 30px;
            opacity: 0.8;
        }
        
        .status {
            padding: 10px 20px;
            border-radius: 25px;
            margin-bottom: 30px;
            font-weight: bold;
            background: rgba(46, 204, 113, 0.3);
            border: 2px solid #2ecc71;
        }
        
        .joystick-container {
            position: relative;
            width: 250px;
            height: 250px;
            margin: 0 auto 30px;
            background: radial-gradient(circle, rgba(255, 255, 255, 0.1) 0%, rgba(0, 0, 0, 0.1) 100%);
            border-radius: 50%;
            border: 3px solid rgba(255, 255, 255, 0.3);
            box-shadow: inset 0 0 20px rgba(0, 0, 0, 0.3);
        }
        
        .joystick-area {
            width: 100%;
            height: 100%;
            border-radius: 50%;
            position: relative;
            cursor: pointer;
        }
        
        .joystick-knob {
            position: absolute;
            width: 60px;
            height: 60px;
            background: linear-gradient(145deg, #ff6b6b, #ee5a52);
            border-radius: 50%;
            top: 50%;
            left: 50%;
            transform: translate(-50%, -50%);
            cursor: pointer;
            box-shadow: 0 4px 15px rgba(238, 90, 82, 0.4);
            border: 3px solid rgba(255, 255, 255, 0.3);
            transition: all 0.1s ease;
        }
        
        .joystick-knob:active {
            transform: translate(-50%, -50%) scale(1.1);
            box-shadow: 0 6px 20px rgba(238, 90, 82, 0.6);
        }
        
        .crosshair {
            position: absolute;
            top: 50%;
            left: 50%;
            transform: translate(-50%, -50%);
            pointer-events: none;
        }
        
        .crosshair::before,
        .crosshair::after {
            content: '';
            position: absolute;
            background: rgba(255, 255, 255, 0.3);
        }
        
        .crosshair::before {
            width: 200px;
            height: 2px;
            top: -1px;
            left: -100px;
        }
        
        .crosshair::after {
            width: 2px;
            height: 200px;
            top: -100px;
            left: -1px;
        }
        
        .direction-labels {
            position: absolute;
            width: 100%;
            height: 100%;
            pointer-events: none;
        }
        
        .direction-label {
            position: absolute;
            font-weight: bold;
            font-size: 14px;
            opacity: 0.7;
        }
        
        .direction-label.north { top: 10px; left: 50%; transform: translateX(-50%); }
        .direction-label.south { bottom: 10px; left: 50%; transform: translateX(-50%); }
        .direction-label.west { left: 10px; top: 50%; transform: translateY(-50%); }
        .direction-label.east { right: 10px; top: 50%; transform: translateY(-50%); }
        
        .velocity-display {
            display: grid;
            grid-template-columns: 1fr 1fr;
            gap: 20px;
            margin-bottom: 30px;
        }
        
        .velocity-item {
            background: rgba(255, 255, 255, 0.1);
            padding: 15px;
            border-radius: 10px;
            border: 1px solid rgba(255, 255, 255, 0.2);
        }
        
        .velocity-label {
            font-size: 0.9em;
            opacity: 0.8;
            margin-bottom: 5px;
        }
        
        .velocity-value {
            font-size: 1.5em;
            font-weight: bold;
        }
        
        .controls {
            display: flex;
            gap: 15px;
            justify-content: center;
            flex-wrap: wrap;
        }
        
        .control-btn {
            padding: 12px 24px;
            border: none;
            border-radius: 25px;
            font-weight: bold;
            cursor: pointer;
            transition: all 0.3s ease;
            text-transform: uppercase;
            letter-spacing: 1px;
        }
        
        .stop-btn {
            background: linear-gradient(145deg, #e74c3c, #c0392b);
            color: white;
            font-size: 16px;
        }
        
        .stop-btn:hover {
            transform: translateY(-2px);
            box-shadow: 0 5px 15px rgba(231, 76, 60, 0.4);
        }
        
        .speed-control {
            margin-top: 20px;
            text-align: center;
        }
        
        .speed-slider {
            width: 200px;
            margin: 10px;
            -webkit-appearance: none;
            height: 8px;
            border-radius: 5px;
            background: rgba(255, 255, 255, 0.2);
            outline: none;
        }
        
        .speed-slider::-webkit-slider-thumb {
            -webkit-appearance: none;
            appearance: none;
            width: 20px;
            height: 20px;
            border-radius: 50%;
            background: #ff6b6b;
            cursor: pointer;
        }
        
        .instructions {
            margin-top: 20px;
            font-size: 0.9em;
            opacity: 0.7;
            line-height: 1.4;
        }
    </style>
</head>
<body>
    <div class="controller">
        <div class="title">🚗 Warehouse Robot</div>
        <div class="subtitle">Joystick Controller</div>
        
        <div class="status" id="status">
            🟢 Connected
        </div>
        
        <div class="joystick-container">
            <div class="joystick-area" id="joystickArea">
                <div class="crosshair"></div>
                <div class="direction-labels">
                    <div class="direction-label north">↑ FORWARD</div>
                    <div class="direction-label south">↓ BACK</div>
                    <div class="direction-label west">← LEFT</div>
                    <div class="direction-label east">RIGHT →</div>
                </div>
                <div class="joystick-knob" id="joystickKnob"></div>
            </div>
        </div>
        
        <div class="velocity-display">
            <div class="velocity-item">
                <div class="velocity-label">Linear Speed</div>
                <div class="velocity-value" id="linearVel">0.00 m/s</div>
            </div>
            <div class="velocity-item">
                <div class="velocity-label">Angular Speed</div>
                <div class="velocity-value" id="angularVel">0.00 rad/s</div>
            </div>
        </div>
        
        <div class="controls">
            <button class="control-btn stop-btn" onclick="emergencyStop()">
                🛑 EMERGENCY STOP
            </button>
        </div>
        
        <div class="speed-control">
            <label>Max Speed: <span id="maxSpeedLabel">0.5</span> m/s</label><br>
            <input type="range" class="speed-slider" id="maxSpeedSlider" 
                   min="0.1" max="1.5" step="0.1" value="0.5" 
                   oninput="updateMaxSpeed(this.value)">
        </div>
        
        <div class="instructions">
            <strong>Instructions:</strong><br>
            • Drag the red knob to control the robot<br>
            • Release to stop movement<br>
            • Use emergency stop for immediate halt<br>
            • Adjust max speed with slider
        </div>
    </div>

    <script>
        class JoystickController {
            constructor() {
                this.joystickArea = document.getElementById('joystickArea');
                this.joystickKnob = document.getElementById('joystickKnob');
                this.maxSpeed = 0.5;
                this.isDragging = false;
                this.centerX = 125;
                this.centerY = 125;
                this.maxDistance = 95;
                
                this.setupEventListeners();
                this.resetKnob();
            }
            
            setupEventListeners() {
                // Mouse events
                this.joystickKnob.addEventListener('mousedown', this.startDrag.bind(this));
                document.addEventListener('mousemove', this.drag.bind(this));
                document.addEventListener('mouseup', this.endDrag.bind(this));
                
                // Touch events for mobile
                this.joystickKnob.addEventListener('touchstart', this.startDrag.bind(this));
                document.addEventListener('touchmove', this.drag.bind(this));
                document.addEventListener('touchend', this.endDrag.bind(this));
            }
            
            startDrag(e) {
                this.isDragging = true;
                e.preventDefault();
            }
            
            drag(e) {
                if (!this.isDragging) return;
                
                e.preventDefault();
                const rect = this.joystickArea.getBoundingClientRect();
                
                let clientX, clientY;
                if (e.type.includes('touch')) {
                    clientX = e.touches[0].clientX;
                    clientY = e.touches[0].clientY;
                } else {
                    clientX = e.clientX;
                    clientY = e.clientY;
                }
                
                const x = clientX - rect.left;
                const y = clientY - rect.top;
                
                this.updateKnobPosition(x, y);
            }
            
            endDrag() {
                if (!this.isDragging) return;
                
                this.isDragging = false;
                this.resetKnob();
            }
            
            updateKnobPosition(x, y) {
                const dx = x - this.centerX;
                const dy = y - this.centerY;
                const distance = Math.sqrt(dx * dx + dy * dy);
                
                let finalX = x;
                let finalY = y;
                
                // Constrain to circle
                if (distance > this.maxDistance) {
                    const angle = Math.atan2(dy, dx);
                    finalX = this.centerX + this.maxDistance * Math.cos(angle);
                    finalY = this.centerY + this.maxDistance * Math.sin(angle);
                }
                
                // Update knob position
                this.joystickKnob.style.left = (finalX - 30) + 'px';
                this.joystickKnob.style.top = (finalY - 30) + 'px';
                
                // Calculate velocities
                const normalizedX = (finalX - this.centerX) / this.maxDistance;
                const normalizedY = (finalY - this.centerY) / this.maxDistance;
                
                const linearVel = -normalizedY * this.maxSpeed; // Invert Y
                const angularVel = -normalizedX * this.maxSpeed * 2; // More sensitive rotation
                
                this.sendVelocityCommand(linearVel, angularVel);
                this.updateDisplay(linearVel, angularVel);
            }
            
            resetKnob() {
                this.joystickKnob.style.left = (this.centerX - 30) + 'px';
                this.joystickKnob.style.top = (this.centerY - 30) + 'px';
                this.sendVelocityCommand(0, 0);
                this.updateDisplay(0, 0);
            }
            
            sendVelocityCommand(linear, angular) {
                fetch('http://localhost:8080/cmd_vel', {
                    method: 'POST',
                    headers: {
                        'Content-Type': 'application/json',
                    },
                    body: JSON.stringify({
                        linear_x: linear,
                        angular_z: angular
                    })
                })
                .then(response => response.json())
                .then(data => {
                    document.getElementById('status').innerHTML = '🟢 Connected';
                    document.getElementById('status').style.background = 'rgba(46, 204, 113, 0.3)';
                    document.getElementById('status').style.borderColor = '#2ecc71';
                })
                .catch(error => {
                    document.getElementById('status').innerHTML = '🔴 Connection Error';
                    document.getElementById('status').style.background = 'rgba(231, 76, 60, 0.3)';
                    document.getElementById('status').style.borderColor = '#e74c3c';
                });
            }
            
            updateDisplay(linear, angular) {
                document.getElementById('linearVel').textContent = linear.toFixed(2) + ' m/s';
                document.getElementById('angularVel').textContent = angular.toFixed(2) + ' rad/s';
            }
            
            setMaxSpeed(speed) {
                this.maxSpeed = speed;
            }
        }
        
        // Initialize joystick
        const joystick = new JoystickController();
        
        function emergencyStop() {
            joystick.resetKnob();
        }
        
        function updateMaxSpeed(value) {
            joystick.setMaxSpeed(parseFloat(value));
            document.getElementById('maxSpeedLabel').textContent = value;
        }
    </script>
</body>
</html>"""
    
    return html_content

def run_web_server(robot_controller, port=8080):
    """Run the web server for the joystick interface"""
    
    # Create the HTML file
    html_content = create_joystick_html()
    with open('joystick.html', 'w') as f:
        f.write(html_content)
    
    # Create custom handler with robot controller
    def handler_factory(*args, **kwargs):
        return JoystickHTTPHandler(*args, robot_controller=robot_controller, **kwargs)
    
    # Start web server
    with socketserver.TCPServer(("", port), handler_factory) as httpd:
        print(f"🌐 Web server running at http://localhost:{port}")
        print(f"🎮 Opening joystick interface...")
        
        # Open web browser
        webbrowser.open(f'http://localhost:{port}/joystick.html')
        
        try:
            httpd.serve_forever()
        except KeyboardInterrupt:
            print("\n🛑 Web server stopping...")
        finally:
            # Clean up HTML file
            if os.path.exists('joystick.html'):
                os.remove('joystick.html')

def main(args=None):
    rclpy.init(args=args)
    
    try:
        # Create robot controller
        robot_controller = WebJoystickController()
        
        # Start ROS2 spinning in background thread
        def ros_spin():
            rclpy.spin(robot_controller)
            
        ros_thread = threading.Thread(target=ros_spin)
        ros_thread.daemon = True
        ros_thread.start()
        
        print("🚗 Warehouse Robot Web Joystick Controller")
        print("=========================================")
        print("✅ ROS2 node started")
        print("🌐 Starting web server...")
        
        # Start web server (this blocks)
        run_web_server(robot_controller)
        
    except KeyboardInterrupt:
        print("\n👋 Shutting down...")
    finally:
        if 'robot_controller' in locals():
            robot_controller.publish_velocity(0.0, 0.0)
            robot_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()