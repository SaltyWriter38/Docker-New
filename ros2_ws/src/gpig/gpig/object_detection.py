from pathlib import Path
from typing import Any, Dict, Optional, Tuple

import cv2
import mediapipe as mp
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String

MARGIN = 10
ROW_SIZE = 10
FONT_SIZE = 1
FONT_THICKNESS = 1
TEXT_COLOR = (255, 0, 0)

DEFAULT_MODEL_NAME = "efficientdet_lite2.tflite"


def visualize(
  image: np.ndarray,
  detection_result: Any,
  box_threshold: float,
  detection_threshold: float,
  distance_from: float,
  max_box_size: float,
  bias_value: float,
) -> Tuple[np.ndarray, Dict[str, Any], np.ndarray]:
  """Draw detections and compute a safe area from obstacle distance transform."""
  object_count = len(detection_result.detections)

  for detection in detection_result.detections:
    if not detection.categories:
      continue

    score = round(detection.categories[0].score, 2)
    if score < box_threshold:
      continue

    bbox = detection.bounding_box
    start_point = int(bbox.origin_x), int(bbox.origin_y)
    end_point = int(bbox.origin_x + bbox.width), int(bbox.origin_y + bbox.height)
    cv2.rectangle(image, start_point, end_point, TEXT_COLOR, 3)

    
    category_name = detection.categories[0].category_name
    result_text = f"{category_name} ({score})"
    text_location = (MARGIN + int(bbox.origin_x), MARGIN + ROW_SIZE + int(bbox.origin_y))
    """
    cv2.putText(
      image,
      result_text,
      text_location,
      cv2.FONT_HERSHEY_PLAIN,
      FONT_SIZE,
      TEXT_COLOR,
      FONT_THICKNESS,
    )
    """

  mask = np.zeros(image.shape[:2], dtype=np.uint8)
  for detection in detection_result.detections:
    if not detection.categories:
      continue

    score = round(detection.categories[0].score, 2)
    if score < detection_threshold:
      continue

    x1 = int(detection.bounding_box.origin_x)
    y1 = int(detection.bounding_box.origin_y)
    x2 = int(detection.bounding_box.origin_x + detection.bounding_box.width)
    y2 = int(detection.bounding_box.origin_y + detection.bounding_box.height)
    cv2.rectangle(mask, (x1, y1), (x2, y2), 255, -1)

  gray_mask = mask.astype(np.uint8)
  distance_map = cv2.distanceTransform(255 - gray_mask, cv2.DIST_L2, 5)

  rows, cols = gray_mask.shape
  x_grid, y_grid = np.ogrid[:rows, :cols]
  center_x, center_y = rows // 2, cols // 2
  distance_from_center = np.sqrt((x_grid - center_x) ** 2 + (y_grid - center_y) ** 2)
  max_distance = np.sqrt(center_x ** 2 + center_y ** 2)

  if max_distance == 0:
    bias_mask = np.ones_like(distance_map)
  else:
    bias_mask = 1.0 - (distance_from_center / max_distance)
    bias_mask = np.power(np.clip(bias_mask, 0.0, 1.0), bias_value)

  weighted_distance_map = distance_map * bias_mask
  _, max_val, _, max_loc = cv2.minMaxLoc(weighted_distance_map)
  space_clearance = float(distance_map[max_loc[1], max_loc[0]])

  meta: Dict[str, Any] = {
    "object_count": object_count,
    "safe_spot_found": False,
    "safe_spot_center": [int(max_loc[0]), int(max_loc[1])],
    "safe_spot_clearance": float(space_clearance),
  }

  if space_clearance < distance_from:
    return image, meta, weighted_distance_map

  usable_half_diagonal = min(space_clearance, max_box_size)
  side = int(usable_half_diagonal * 1.414)
  x = int(max_loc[0] - side // 2)
  y = int(max_loc[1] - side // 2)

  x1 = max(0, x)
  y1 = max(0, y)
  x2 = min(image.shape[1], x + side)
  y2 = min(image.shape[0], y + side)
  cv2.rectangle(image, (x1, y1), (x2, y2), (0, 255, 0), 3)

  meta["safe_spot_found"] = True
  meta["safe_spot_box"] = [x1, y1, x2, y2]
  return image, meta, weighted_distance_map


class ObjectDetectionNode(Node):
  def __init__(self) -> None:
    super().__init__("object_detection")
    self.bridge = CvBridge()

    self.declare_parameter("image_topic", "/camera")
    self.declare_parameter("annotated_topic", "/gpig/object_detection/annotated")
    self.declare_parameter("summary_topic", "/gpig/object_detection/summary")
    self.declare_parameter("model_name", DEFAULT_MODEL_NAME)
    self.declare_parameter("model_path", "")
    self.declare_parameter("max_detection_results", 100)
    self.declare_parameter("box_threshold", 0.1)
    self.declare_parameter("detection_threshold", 0.1)
    self.declare_parameter("distance_from", 3.0)
    self.declare_parameter("max_box_size", 400.0)
    self.declare_parameter("bias_value", 2.0)
    self.declare_parameter("show_debug_windows", False)

    self.image_topic = str(self.get_parameter("image_topic").value)
    self.annotated_topic = str(self.get_parameter("annotated_topic").value)
    self.summary_topic = str(self.get_parameter("summary_topic").value)
    self.model_name = str(self.get_parameter("model_name").value)
    self.model_path_override = str(self.get_parameter("model_path").value)
    self.max_detection_results = int(self.get_parameter("max_detection_results").value)
    self.box_threshold = float(self.get_parameter("box_threshold").value)
    self.detection_threshold = float(self.get_parameter("detection_threshold").value)
    self.distance_from = float(self.get_parameter("distance_from").value)
    self.max_box_size = float(self.get_parameter("max_box_size").value)
    self.bias_value = float(self.get_parameter("bias_value").value)
    self.show_debug_windows = bool(self.get_parameter("show_debug_windows").value)

    resolved_model_path = self._resolve_model_path()
    self.detector = self._build_detector(resolved_model_path)

    self.annotated_pub = self.create_publisher(Image, self.annotated_topic, 10)
    self.summary_pub = self.create_publisher(String, self.summary_topic, 10)
    self.image_sub = self.create_subscription(Image, self.image_topic, self.image_callback, 10)

    self.get_logger().info(f"Object detector ready with model: {resolved_model_path}")

  def _resolve_model_path(self) -> str:
    if self.model_path_override:
      candidate = Path(self.model_path_override)
      if candidate.is_file():
        return str(candidate)
      raise FileNotFoundError(f"model_path does not exist: {candidate}")

    share_dir = Path(get_package_share_directory("gpig"))
    packaged_model = share_dir / "models" / self.model_name
    if packaged_model.is_file():
      return str(packaged_model)

    source_model = Path(__file__).resolve().parent / "models" / self.model_name
    if source_model.is_file():
      return str(source_model)

    raise FileNotFoundError(
      f"Could not find model '{self.model_name}' in installed share or source tree"
    )

  def _build_detector(self, model_path: str) -> Any:
    base_options = mp.tasks.BaseOptions
    object_detector = mp.tasks.vision.ObjectDetector
    object_detector_options = mp.tasks.vision.ObjectDetectorOptions
    vision_running_mode = mp.tasks.vision.RunningMode

    options = object_detector_options(
      base_options=base_options(model_asset_path=model_path),
      max_results=self.max_detection_results,
      running_mode=vision_running_mode.IMAGE,
    )
    return object_detector.create_from_options(options)

  def image_callback(self, msg: Image) -> None:
    frame_bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
    frame_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
    mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=frame_rgb)

    detection_result = self.detector.detect(mp_image)
    annotated, meta, weighted_map = visualize(
      frame_bgr.copy(),
      detection_result,
      self.box_threshold,
      self.detection_threshold,
      self.distance_from,
      self.max_box_size,
      self.bias_value,
    )

    annotated_msg = self.bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
    annotated_msg.header = msg.header
    self.annotated_pub.publish(annotated_msg)

    summary = String()
    summary.data = (
      f"objects={meta['object_count']} "
      f"safe_spot_found={meta['safe_spot_found']} "
      f"safe_spot_center={meta['safe_spot_center']} "
      f"safe_spot_clearance={meta['safe_spot_clearance']:.3f}"
    )
    self.summary_pub.publish(summary)

    if self.show_debug_windows:
      heatmap = cv2.normalize(weighted_map, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8U)
      cv2.imshow("Object Detection", annotated)
      cv2.imshow("Safety Heatmap", heatmap)
      cv2.waitKey(1)

  def destroy_node(self) -> bool:
    if self.show_debug_windows:
      cv2.destroyAllWindows()
    self.detector.close()
    return super().destroy_node()


def main(args: Optional[list] = None) -> None:
  rclpy.init(args=args)
  node = ObjectDetectionNode()
  try:
    rclpy.spin(node)
  finally:
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
  main()
    
#How to run

#Build and source:
#cd /home/developer/workspace/ros2_ws
#colcon build --packages-select gpig
#source install/setup.bash

#Run node:
#ros2 run gpig object_detection

#Optional parameter override example:
#ros2 run gpig object_detection --ros-args -p image_topic:=/camera/image_raw -p show_debug_windows:=true -p model_name:=efficientdet_lite2.tflite