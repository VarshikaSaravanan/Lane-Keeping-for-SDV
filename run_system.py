
import sys
import logging
import cv2
from backend.core import LaneKeepingSystem
from backend.metrics import MetricTracker
from frontend.ui import Visualizer

# Configure Logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(message)s')

def main():
    video_source = "challenge_video.mp4"
    
    print(f"Initializing SDV System on {video_source}...")
    
    # Initialize Backend and Frontend
    system = LaneKeepingSystem(video_source)
    tracker = MetricTracker()
    ui = Visualizer("SDV Lane Assist")
    
    if not system.cap or not system.cap.isOpened():
        logging.error("Failed to initialize video. check 'project_video.mp4'.")
        return

    while True:
        ret, frame = system.get_frame()
        if not ret:
            break
            
        # Backend Process
        processed_frame, angle, ldw, jerk, speed, brake = system.process(frame)
        
        # Update Metrics
        tracker.update(angle)
        
        # Frontend Render
        ui.render(processed_frame, angle, ldw, jerk, speed, brake, tracker.get_data())
        
        if cv2.waitKey(25) & 0xFF == ord('q'):
            break
            
    system.cleanup()
    ui.close()
    print("Simulation finished.")

if __name__ == "__main__":
    main()
