import cv2
import apriltag
import numpy as np
import rclpy
from .Robot import Robot
from ultralytics import YOLO

class Camera:
    def __init__(self, robot : Robot):
        self.robot = robot
        self.model = None


    def checkImage(self) -> np.ndarray:
        ''' Waits for the robot\'s image future to complete, then returns the latest image message. '''
        self.robot.image_future = rclpy.Future()
        try:
            self.robot.spin_until_future_completed(self.robot.image_future)
        except Exception as e:
            # Log the exception and return None when the node is shutting down or a KeyboardInterrupt occurs.
            print("Exception in checkImage:", e)
            return None
        return self.robot.last_image_msg

    def checkImageRelease(self): #this one returns an actual image instead of all the data
        ''' Retrieves the latest image message, reshapes its data into a 3D image array, and displays it using OpenCV. '''
        image = self.checkImage()
        height = image.height
        width = image.width
        img_data = image.data
        img_3D = np.reshape(img_data, (height, width, 3))
        cv2.imshow("image", img_3D)
        cv2.waitKey(10)

    def checkCamera(self) -> np.ndarray:
        ''' Waits for and returns the most recent camera info message using the robot\'s camera info future. '''
        self.robot.camera_info_future = rclpy.Future()
        self.robot.spin_until_future_completed(self.robot.camera_info_future)
        return self.robot.last_camera_info_msg 

    def estimate_apriltag_pose(self, image : np.ndarray) -> list[tuple[int, float, float, float]]:
        ''' Converts the image to grayscale, detects AprilTags, and, if successful, estimates the pose. '''
        # Convert image to grayscale for tag detection
        img_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        detections = apriltag.Detector(apriltag.DetectorOptions(families="tag16h5, tag25h9")).detect(img_gray)

        # If camera calibration matrix is not available or no detections, return empty list
        if not detections or self.robot.k is None:
            return []
        
        
        poses = []
        half_size = self.robot.TAG_SIZE / 2.0
        object_points = np.array([
            [-half_size,  half_size, 0],
            [ half_size,  half_size, 0],
            [ half_size, -half_size, 0],
            [-half_size, -half_size, 0]
        ], dtype=np.float32)

        # Process each detected tag
        for detection in detections:
            tag_id = detection.tag_id
            image_points = np.array(detection.corners, dtype=np.float32)

            # Estimate position
            ret, rvec, tvec = cv2.solvePnP(object_points, image_points, self.robot.k, None)
            if not ret:
                continue

            tvec = tvec.flatten()
            range_ = np.linalg.norm(tvec)
            bearing = np.degrees(np.arctan2(tvec[0], tvec[2]))
            elevation = np.degrees(np.arctan2(tvec[1], tvec[2]))

            poses.append((tag_id, range_, bearing, elevation))

        return poses
    
    def rosImg_to_cv2(self) -> np.ndarray:
        ''' Retrieves the current ROS image message, reshapes its raw data into three-channel image, and returns it. '''
        image = self.checkImage()
        if image is None:
            # Gracefully handle the case when no image was received.
            return None
        height = image.height
        width = image.width
        img_data = image.data
        img_3D = np.reshape(img_data, (height, width, 3))
        return img_3D

    def ML_predict_stop_sign(self, img : np.ndarray) -> tuple[bool, int, int, int, int]:
        ''' Uses the provided ML model to predict the presence of a stop sign within the image, draws a bounding box around any detection, displays the result, and returns both the detection flag and bounding box coords. '''
        if self.model == None:
            self.model = YOLO("yolov8n.pt")  # Load the YOLOv8 model
        
        stop_sign_detected = False

        x1 = -1
        y1 = -1 
        x2 = -1 
        y2 = -1

        # Predict stop signs in image using model
        results = self.model.predict(img, classes=[11], conf=0.25, imgsz=640, max_det=1, verbose=False)
        
        # Results is a list containing the results object with all data
        results_obj = results[0]
        
        # Extract bounding boxes
        boxes = results_obj.boxes.xyxy

        try:
            for box in boxes:
                x1, y1, x2, y2 = map(int, box[:4])
                cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), 3)
                stop_sign_detected = True
        except:
            stop_sign_detected = False

        # cv2.imshow("Bounding Box", img)

        return stop_sign_detected, x1, y1, x2, y2   
