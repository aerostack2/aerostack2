# One object detected by the perception pipeline, with its class, image location and optional 3D pose.

std_msgs/Header header                              # frame of detection
vision_msgs/ObjectHypothesisWithPose hypothesis     # detection : class/id, confidence and its pose
vision_msgs/BoundingBox2D bbox                      # box around the detected object in the image (pixels)
vision_msgs/Point2D[] keypoints                     # detected keypoints of the object
string[] keypoint_names                             # name of each detected keypoint (e.g. "top_left")
float32[] keypoint_scores                           # confidence of each detected keypoint
bool pose_valid                                     # true when hypothesis.pose holds a valid 3D pose, 
                                                    #    false for 2D detections
