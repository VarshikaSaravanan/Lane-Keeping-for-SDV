
import numpy as np

class LateralController:
    def __init__(self, max_steering_angle=30.0, max_steering_rate=5.0):
        """
        Controller with Jerk Minimization.
        
        Args:
            max_steering_angle (float): Max wheel angle in degrees.
            max_steering_rate (float): Max change in angle per frame (Safety/Comfort Limit).
        """
        self.max_steering_angle = max_steering_angle
        self.max_steering_rate = max_steering_rate # This limits Jerk directly
        self.prev_steering_angle = 0.0
        
        # Safety Flags
        self.ldw_active = False
        self.jerk_warning = False

    def compute_steering(self, offset_meters, curvature_radius):
        """
        Compute safe, jerk-minimized steering angle.
        """
        # 1. Pure Pursuit / P-Control logic
        # Ideally: steering ~ atan(2*L*sin(alpha)/dist)
        # Simplified for this simulation: Target Angle proportional to CTE (Cross Track Error)
        
        Kp = 20.0 # Gain
        target_angle = offset_meters * Kp
        
        # 2. Hard Limits (Physical Car Limit)
        target_angle = np.clip(target_angle, -self.max_steering_angle, self.max_steering_angle)
        
        # 3. Jerk Minimization (Slew Rate Limiter)
        # We limit how fast the steering wheel can turn between frames.
        delta = target_angle - self.prev_steering_angle
        
        if abs(delta) > self.max_steering_rate:
            # CAP the change to ensure smoothness
            delta = np.sign(delta) * self.max_steering_rate
            self.jerk_warning = True # Flag that we are limiting "Jerk"
        else:
            self.jerk_warning = False
            
        final_angle = self.prev_steering_angle + delta
        self.prev_steering_angle = final_angle
        
        # 4. Lane Departure Warning (LDW) Check
        # If car is > 0.6m from center, trigger warning
        if abs(offset_meters) > 0.6:
            self.ldw_active = True
        else:
            self.ldw_active = False
            
        return final_angle, self.ldw_active, self.jerk_warning

class LongitudinalController:
    def __init__(self, max_speed_kph=100.0):
        """
        Controls Vehicle Speed based on Safety/Curvature.
        """
        self.max_speed = max_speed_kph
        self.current_speed = max_speed_kph
        self.brake_active = False
        
    def compute_speed(self, curvature_radius):
        """
        Slow down for curves. 
        radius < 500m -> Slow Down.
        """
        # Safety Logic: Sharper curve (small radius) = Lower Speed
        # Simple Physics rule: v^2 / r < Max_Lateral_G
        
        target_speed = self.max_speed
        
        # If radius is small (e.g. < 800m), we need to slow down
        # Radius can be huge (10000m) for straight roads, so we clamp it
        safe_radius = min(curvature_radius, 2000.0)
        
        if safe_radius < 800:
            # Curve detected!
            # Mapping: 800m -> 100kph, 200m -> 40kph
            factor = (safe_radius - 200) / 600 # 0.0 to 1.0
            factor = np.clip(factor, 0.0, 1.0)
            
            target_speed = 40 + (factor * 60) # Linear interpolation
            self.brake_active = True
        else:
            self.brake_active = False
            
        # Smooth speed change (Inertia)
        alpha = 0.1 # Smoothing factor
        self.current_speed = (self.current_speed * (1-alpha)) + (target_speed * alpha)
        
        return self.current_speed, self.brake_active
