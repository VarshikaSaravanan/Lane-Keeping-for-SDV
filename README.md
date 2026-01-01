# Lane Keeping for SDV
🚗 Software-Defined Vehicle: Lane Keeping & Autonomous Safety System
A pure-software implementation of an Autonomous Driving ADAS (Advanced Driver Assistance System) using Python and OpenCV.

This project simulates a Level 2 Autonomous Vehicle capable of robust Lane Keeping and Adaptive Speed Control, replacing traditional MATLAB/Simulink workflows with a flexible, open-source stack.

🌟 Key Features
1. Advanced Perception Stack (Computer Vision)
Lane Detection: Robust lane tracking using HLS Color Thresholding, Sliding Window Search, and 2nd-Degree Polynomial Fitting.
Bird's Eye View: Inverse Perspective Mapping (IPM) for accurate road curvature measurement ($R_{curve}$) and lane positioning.
Real-World Mapping: Converts pixel data to meters to calculate precise lateral offset.
2. Control Systems (Safety & Comfort)
Lateral Control (Steering):
Jerk Minimization: Implements a Slew Rate Limiter to strictly control the rate of steering change, ensuring passenger comfort (ISO 2631 standards).
PID Logic: Smoothly centers the vehicle using real-time error correction.
Longitudinal Control (Speed):
Curve Speed Adaptation: Automatically slows the vehicle down for sharp curves based on physics checks ($v^2/R$), preventing lateral slip.
3. Safety & Visualization
Lane Departure Warning (LDW): Audio/Visual alerts if the vehicle drifts more than 0.6m from the lane center.
Real-Time Dashboard: Displays Steering Angle, Speed, Curvature Radius, and Live Telemetry Graphs (Steering History & Jerk Profile).
🛠️ Tech Stack
Core: Python 3.9+
Vision: OpenCV, NumPy
Data: Matplotlib (Telemetry)
🚀 How It Works
The system follows a modular Perception-Plan-Act pipeline:

Input: Dashcam video feed.
Perception: Extracts lane lines and maps them to a bird's eye view.
Planning: Calculates the ideal steering angle and safe max speed for the current road curvature.
Control: Smooths the inputs to prevent jerky maneuvers and maintain passenger comfort.
Simulate: Overlays the decision data and safety warnings onto the video feed.
