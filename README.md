🚗 Software-Defined Vehicle: Lane Keeping & Autonomous Safety System
A pure-software implementation of an Autonomous Driving ADAS (Advanced Driver Assistance System) using Python and OpenCV.

This project simulates a Level 2 Autonomous Vehicle capable of Lane Keeping, Adaptive Speed Control, and Traffic Sign Recognition without relying on expensive proprietary tools like MATLAB/Simulink.

🌟 Key Features
1. Advanced Perception Stack (Computer Vision)
Lane Detection: Robust lane tracking using HLS Color Thresholding, Sliding Window Search, and 2nd-Degree Polynomial Fitting.
Bird's Eye View: Inverse Perspective Mapping (IPM) for accurate road curvature measurement ($R_{curve}$).
Object Detection:
YOLOv8 Integration: Real-time detection of Stop Signs and Traffic Lights.
Signal Classification: Color-based logic to distinguish Red vs Green lights.
Custom CV Fallback: HSV-based detection for Yellow Curve Warning signs.
2. Control Systems (Safety & Comfort)
Lateral Control (Steering):
Implements a Slew Rate Limiter to minimize "Jerk" (lateral acceleration change), ensuring passenger comfort (ISO 2631 standards).
Real-time steering smoothing based on path curvature.
Longitudinal Control (Speed):
Adaptive Cruise Control: Automatically brakes for sharp curves ($v^2/R$ physics checks) and visible Stop/Warning signs.
3. Safety & Visualization
Lane Departure Warning (LDW): Alerts driver if deviations exceed 0.6m.
Real-Time Dashboard: Displays Steering Angle, Speed, Brake Status, and Live Telemetry Graphs (Steering History & Jerk Profile).
🛠️ Tech Stack
Core: Python 3.9+
Vision: OpenCV, NumPy, Ultralytics (YOLOv8)
Data: Matplotlib (Telemetry)
🚀 How It Works
The system follows a modular Perception-Plan-Act pipeline:

Input: Dashcam video feed.
Perception: Extracts lane lines and detects road objects.
Planning: Calculates ideal steering angle and safe speed limit.
Control: smooths the inputs to prevent jerky maneuvers.
Simulate: Overlays the decision data onto the video feed.
