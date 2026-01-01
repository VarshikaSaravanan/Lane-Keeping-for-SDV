
import cv2
import numpy as np
import logging
from backend.control import LateralController, LongitudinalController

class LaneKeepingSystem:
    def __init__(self, video_path):
        self.video_path = video_path
        self.cap = cv2.VideoCapture(video_path)
        if not self.cap.isOpened():
             logging.error(f"Could not open video: {video_path}")
        
        # Initialize Controller
        self.controller = LateralController()
        self.speed_controller = LongitudinalController(max_speed_kph=100.0)
        
    def get_frame(self):
        """
        Get the next frame from source.
        """
        if self.cap.isOpened():
            return self.cap.read()
        return False, None

    def process(self, frame):
        """
        Core Logic Pipeline.
        """
        # 1. Perception
        masked_edges = self._detect_lanes_color_gradient(frame)
        warped_edges = self._warp_to_birdseye(masked_edges)
        
        # 2. Lane Fitting (Sliding Windows)
        left_fit, right_fit, ploty = self._fit_polynomial(warped_edges)
        
        if left_fit is not None and right_fit is not None:
            curvature, offset_meter = self._measure_curvature_real(ploty, left_fit, right_fit)
            
            # 4. Control (Jerk Minimized)
            steering_angle, ldw, jerk_warn = self.controller.compute_steering(offset_meter, curvature)
            speed, braking = self.speed_controller.compute_speed(curvature)
            
            # 5. Visualization
            result_frame = self._draw_lane_overlay(frame, warped_edges, left_fit, right_fit)
            
            # Return extra status for UI
            return result_frame, steering_angle, ldw, jerk_warn, speed, braking
        else:
            steering_angle = 0.0
            # No detection - Safety Fallback (Cruise, no brake)
            return frame, steering_angle, False, False, 100.0, False

    def _warp_to_birdseye(self, img):
        height, width = img.shape[:2]
        src = np.float32([[width * 0.43, height * 0.65], [width * 0.57, height * 0.65], 
                          [width * 0.10, height], [width * 0.90, height]])
        dst = np.float32([[width * 0.25, 0], [width * 0.75, 0], 
                          [width * 0.25, height], [width * 0.75, height]])
        self.M = cv2.getPerspectiveTransform(src, dst)
        self.Minv = cv2.getPerspectiveTransform(dst, src) # Inverse for unwarping
        return cv2.warpPerspective(img, self.M, (width, height))

    def _fit_polynomial(self, binary_warped):
        """
        Find lane pixels using sliding windows and fit a 2nd order polynomial.
        """
        # Histogram to find starting points
        histogram = np.sum(binary_warped[binary_warped.shape[0]//2:,:], axis=0)
        midpoint = int(histogram.shape[0]//2)
        leftx_base = np.argmax(histogram[:midpoint])
        rightx_base = np.argmax(histogram[midpoint:]) + midpoint

        # Sliding Window Hyperparameters
        nwindows = 9
        margin = 100
        minpix = 50
        window_height = int(binary_warped.shape[0]//nwindows)
        
        # Identify non-zero pixels
        nonzero = binary_warped.nonzero()
        nonzeroy = np.array(nonzero[0])
        nonzerox = np.array(nonzero[1])
        
        # Current positions
        leftx_current = leftx_base
        rightx_current = rightx_base
        
        left_lane_inds = []
        right_lane_inds = []

        for window in range(nwindows):
            win_y_low = binary_warped.shape[0] - (window+1)*window_height
            win_y_high = binary_warped.shape[0] - window*window_height
            
            # Find pixels in window
            good_left_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & 
                            (nonzerox >= leftx_current - margin) & (nonzerox < leftx_current + margin)).nonzero()[0]
            good_right_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & 
                            (nonzerox >= rightx_current - margin) & (nonzerox < rightx_current + margin)).nonzero()[0]
            
            left_lane_inds.append(good_left_inds)
            right_lane_inds.append(good_right_inds)
            
            # Recenter if enough pixels found
            if len(good_left_inds) > minpix:
                leftx_current = int(np.mean(nonzerox[good_left_inds]))
            if len(good_right_inds) > minpix:
                rightx_current = int(np.mean(nonzerox[good_right_inds]))

        # Concatenate indices
        left_lane_inds = np.concatenate(left_lane_inds)
        right_lane_inds = np.concatenate(right_lane_inds)

        # Extract pixel positions
        leftx = nonzerox[left_lane_inds]
        lefty = nonzeroy[left_lane_inds] 
        rightx = nonzerox[right_lane_inds]
        righty = nonzeroy[right_lane_inds] 

        # Fit Polynomial: x = Ay^2 + By + C
        if len(leftx) == 0 or len(rightx) == 0:
            return None, None, None

        left_fit = np.polyfit(lefty, leftx, 2)
        right_fit = np.polyfit(righty, rightx, 2)
        
        ploty = np.linspace(0, binary_warped.shape[0]-1, binary_warped.shape[0])
        return left_fit, right_fit, ploty

    def _measure_curvature_real(self, ploty, left_fit, right_fit):
        """
        Calculate curvature in meters and distance from center.
        """
        # Conversions
        ym_per_pix = 30/720 # meters per pixel in y dimension
        xm_per_pix = 3.7/700 # meters per pixel in x dimension
        
        # Define y-value where we want radius of curvature (at bottom)
        y_eval = np.max(ploty)
        
        # Fit new polynomials to x,y in world space
        left_fit_cr = np.polyfit(ploty*ym_per_pix, (left_fit[0]*ploty**2 + left_fit[1]*ploty + left_fit[2])*xm_per_pix, 2)
        right_fit_cr = np.polyfit(ploty*ym_per_pix, (right_fit[0]*ploty**2 + right_fit[1]*ploty + right_fit[2])*xm_per_pix, 2)
        
        # Calculate Radii
        left_curverad = ((1 + (2*left_fit_cr[0]*y_eval*ym_per_pix + left_fit_cr[1])**2)**1.5) / np.absolute(2*left_fit_cr[0])
        right_curverad = ((1 + (2*right_fit_cr[0]*y_eval*ym_per_pix + right_fit_cr[1])**2)**1.5) / np.absolute(2*right_fit_cr[0])
        
        curvature = (left_curverad + right_curverad) / 2
        
        # Calculate Offset from Center
        img_center = 1280 / 2 # Assuming 1280x720
        left_x = left_fit[0]*y_eval**2 + left_fit[1]*y_eval + left_fit[2]
        right_x = right_fit[0]*y_eval**2 + right_fit[1]*y_eval + right_fit[2]
        lane_center = (left_x + right_x) / 2
        offset = (lane_center - img_center) * xm_per_pix
        
        return curvature, offset

    def _draw_lane_overlay(self, original_img, warped, left_fit, right_fit):
        """
        Draw the lane corridor on the original image.
        """
        ploty = np.linspace(0, warped.shape[0]-1, warped.shape[0])
        left_fitx = left_fit[0]*ploty**2 + left_fit[1]*ploty + left_fit[2]
        right_fitx = right_fit[0]*ploty**2 + right_fit[1]*ploty + right_fit[2]

        # Create an image to draw the lines on
        warp_zero = np.zeros_like(warped).astype(np.uint8)
        color_warp = np.dstack((warp_zero, warp_zero, warp_zero))

        # Recast the x and y points into usable format for cv2.fillPoly()
        pts_left = np.array([np.transpose(np.vstack([left_fitx, ploty]))])
        pts_right = np.array([np.flipud(np.transpose(np.vstack([right_fitx, ploty])))])
        pts = np.hstack((pts_left, pts_right))

        # Draw the lane onto the warped blank image (Green)
        cv2.fillPoly(color_warp, np.int_([pts]), (0, 255, 0))

        # Warp the blank back to original image space using inverse perspective matrix (Minv)
        newwarp = cv2.warpPerspective(color_warp, self.Minv, (original_img.shape[1], original_img.shape[0])) 
        
        # Combine the result with the original image
        result = cv2.addWeighted(original_img, 1, newwarp, 0.3, 0)
        return result

    def _detect_lanes_color_gradient(self, frame):
        """
        Robust lane detection using HLS color space and Canny Edge Detection.
        """
        # 1. Convert to HLS (Hue, Lightness, Saturation)
        hls = cv2.cvtColor(frame, cv2.COLOR_BGR2HLS)
        
        # 2. White Color Mask (High Lightness)
        lower_white = np.array([0, 200, 0])
        upper_white = np.array([255, 255, 255])
        white_mask = cv2.inRange(hls, lower_white, upper_white)
        
        # 3. Yellow Color Mask (Specific Hue ~20-30, High Saturation)
        lower_yellow = np.array([15, 30, 115])
        upper_yellow = np.array([35, 204, 255])
        yellow_mask = cv2.inRange(hls, lower_yellow, upper_yellow)
        
        # Combine Masks
        combined_mask = cv2.bitwise_or(white_mask, yellow_mask)
        
        # 4. Mask the original image
        masked_image = cv2.bitwise_and(frame, frame, mask=combined_mask)
        
        # 5. Grayscale + Blur + Canny
        gray = cv2.cvtColor(masked_image, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        edges = cv2.Canny(blur, 50, 150)
        
        # 6. Region of Interest (ROI)
        height, width = edges.shape
        # Define a trapezoid covering the road lane
        polygons = np.array([
            [(0, height), (width, height), (int(width*0.55), int(height*0.6)), (int(width*0.45), int(height*0.6))]
        ])
        
        mask = np.zeros_like(edges)
        cv2.fillPoly(mask, polygons, 255)
        masked_edges = cv2.bitwise_and(edges, mask)
        
        return masked_edges

    def cleanup(self):
        if self.cap:
            self.cap.release()
