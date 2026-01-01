
import cv2
import numpy as np

class Visualizer:
    def __init__(self, window_name="SDV Lane Assist"):
        self.window_name = window_name

    def render(self, frame, steering_angle=0.0, ldw=False, jerk_warn=False, speed=100.0, brake=False, metrics=None):
        """
        Draw overlays and show the frame.
        """
        # Overlay Dashboard
        # Status Bar
        cv2.rectangle(frame, (0, 0), (frame.shape[1], 80), (0, 0, 0), -1)
        
        # Steering INFO
        cv2.putText(frame, f"STEERING: {steering_angle:.2f} deg", (20, 40), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
        
        # Speedometer INFO
        color_speed = (0, 255, 0)
        if brake:
            color_speed = (0, 0, 255) # Red if braking
            cv2.putText(frame, "BRAKING", (250, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            
        cv2.putText(frame, f"SPEED: {int(speed)} km/h", (20, 70), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, color_speed, 2)

        # Safety Alerts
        if ldw:
            cv2.rectangle(frame, (350, 10), (600, 70), (0, 0, 255), -1)
            cv2.putText(frame, "WARNING: LDW", (370, 50), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
                        
        if jerk_warn:
            cv2.rectangle(frame, (620, 10), (900, 70), (0, 165, 255), -1)
            cv2.putText(frame, "COMFORT: JERK LIM", (640, 50), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            
        # Draw Metrics Graphs if available
        if metrics:
            steering_data, jerk_data = metrics
            self._draw_graph(frame, steering_data, "Angle", (50, 600), (0, 255, 0))
            self._draw_graph(frame, jerk_data, "Jerk", (400, 600), (0, 165, 255))
            
        cv2.imshow(self.window_name, frame)

    def _draw_graph(self, img, data, label, position, color):
        """
        Draw a simple line graph.
        """
        x, y = position
        w, h = 300, 100
        
        # Background
        cv2.rectangle(img, (x, y), (x+w, y+h), (0, 0, 0), -1)
        cv2.putText(img, label, (x+5, y+20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        if len(data) < 2:
            return
            
        # Normalize data to fit height
        max_val = max(abs(min(data)), abs(max(data))) + 0.1
        scale = (h / 2) / max_val
        
        pts = []
        step = w / len(data)
        
        for i, val in enumerate(data):
            px = int(x + i * step)
            py = int(y + h/2 - val * scale)
            pts.append((px, py))
            
        for i in range(len(pts) - 1):
            cv2.line(img, pts[i], pts[i+1], color, 2)

    def close(self):
        cv2.destroyAllWindows()
