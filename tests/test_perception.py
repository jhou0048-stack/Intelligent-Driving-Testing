"""Tests for the perception module."""

from unittest.mock import MagicMock, patch

import numpy as np
import pytest

from perception import (
    Camera,
    Detection,
    GazeboCamera,
    GazeboLidar,
    Lidar,
    ObjectDetector,
    Sensor,
    SensorReading,
    YOLODetector,
)
from perception.lidar import ranges_to_point_cloud


# --- ABC tests ---------------------------------------------------------------


def test_sensor_is_abstract() -> None:
    with pytest.raises(TypeError):
        Sensor()  # type: ignore[abstract]


def test_camera_is_abstract() -> None:
    with pytest.raises(TypeError):
        Camera()  # type: ignore[abstract]


def test_lidar_is_abstract() -> None:
    with pytest.raises(TypeError):
        Lidar()  # type: ignore[abstract]


def test_object_detector_is_abstract() -> None:
    with pytest.raises(TypeError):
        ObjectDetector()  # type: ignore[abstract]


# --- SensorReading dataclass --------------------------------------------------


def test_sensor_reading_fields() -> None:
    reading = SensorReading(timestamp=1.5, sensor_name="cam", data=42)
    assert reading.timestamp == 1.5
    assert reading.sensor_name == "cam"
    assert reading.data == 42


def test_sensor_reading_is_frozen() -> None:
    reading = SensorReading(timestamp=1.0, sensor_name="cam", data=None)
    with pytest.raises(AttributeError):
        reading.timestamp = 2.0  # type: ignore[misc]


# --- Detection dataclass -----------------------------------------------------


def test_detection_fields() -> None:
    d = Detection(class_name="car", confidence=0.95, bbox=(10, 20, 100, 200))
    assert d.class_name == "car"
    assert d.confidence == pytest.approx(0.95)
    assert d.bbox == (10, 20, 100, 200)


# --- GazeboCamera -------------------------------------------------------------


def test_gazebo_camera_topic_construction() -> None:
    cam = GazeboCamera(
        model_name="ego_vehicle",
        link_name="camera_link",
        sensor_name="front_camera",
        world_name="simple_road",
    )
    assert cam.topic == (
        "/world/simple_road/model/ego_vehicle"
        "/link/camera_link/sensor/front_camera/image"
    )


def test_gazebo_camera_default_dimensions() -> None:
    cam = GazeboCamera()
    assert cam.width == 640
    assert cam.height == 480


def test_gazebo_camera_owns_simulator_by_default() -> None:
    cam = GazeboCamera()
    assert cam._own_simulator is True


def test_gazebo_camera_external_simulator() -> None:
    sim = MagicMock()
    cam = GazeboCamera(simulator=sim, own_simulator=False)
    assert cam._simulator is sim
    assert cam._own_simulator is False


def test_gazebo_camera_connect_subscribes() -> None:
    sim = MagicMock()
    cam = GazeboCamera(simulator=sim, own_simulator=False)
    cam.connect()
    sim.subscribe.assert_called_once()
    topic, msg_type, _callback = sim.subscribe.call_args[0]
    assert "image" in topic
    assert msg_type == "Image"


def test_gazebo_camera_decode_rgb_image() -> None:
    sim = MagicMock()
    cam = GazeboCamera(
        simulator=sim, own_simulator=False, image_width=2, image_height=2
    )
    msg = MagicMock()
    msg.width = 2
    msg.height = 2
    msg.pixel_format_type = 5  # RGB_INT8
    msg.data = bytes([255, 0, 0, 0, 255, 0, 0, 0, 255, 128, 128, 128])
    msg.header.stamp.sec = 10
    msg.header.stamp.nsec = 500_000_000

    cam._on_image(msg)
    image = cam.get_image()

    assert image is not None
    assert image.shape == (2, 2, 3)
    assert image[0, 0, 2] == 255  # R channel in BGR position
    assert image[0, 0, 0] == 0    # B channel


def test_gazebo_camera_get_latest_returns_reading() -> None:
    sim = MagicMock()
    cam = GazeboCamera(simulator=sim, own_simulator=False)
    msg = MagicMock()
    msg.width = 1
    msg.height = 1
    msg.pixel_format_type = 5
    msg.data = bytes([100, 150, 200])
    msg.header.stamp.sec = 5
    msg.header.stamp.nsec = 0

    cam._on_image(msg)
    reading = cam.get_latest()

    assert reading is not None
    assert reading.sensor_name == "front_camera"
    assert reading.timestamp == pytest.approx(5.0)
    assert isinstance(reading.data, np.ndarray)


def test_gazebo_camera_no_image_returns_none() -> None:
    sim = MagicMock()
    cam = GazeboCamera(simulator=sim, own_simulator=False)
    assert cam.get_image() is None
    assert cam.get_latest() is None


def test_gazebo_camera_context_manager() -> None:
    cam = GazeboCamera()
    with patch.object(cam, "connect") as mock_connect, patch.object(
        cam, "disconnect"
    ) as mock_disconnect:
        with cam:
            mock_connect.assert_called_once()
        mock_disconnect.assert_called_once()


# --- GazeboLidar --------------------------------------------------------------


def test_gazebo_lidar_topic_construction() -> None:
    lidar = GazeboLidar(
        model_name="ego_vehicle",
        link_name="lidar_link",
        sensor_name="front_lidar",
        world_name="simple_road",
    )
    assert lidar.topic == (
        "/world/simple_road/model/ego_vehicle"
        "/link/lidar_link/sensor/front_lidar/scan"
    )


def test_gazebo_lidar_owns_simulator_by_default() -> None:
    lidar = GazeboLidar()
    assert lidar._own_simulator is True


def test_gazebo_lidar_connect_subscribes() -> None:
    sim = MagicMock()
    lidar = GazeboLidar(simulator=sim, own_simulator=False)
    lidar.connect()
    sim.subscribe.assert_called_once()
    topic, msg_type, _callback = sim.subscribe.call_args[0]
    assert "scan" in topic
    assert msg_type == "LaserScan"


def test_gazebo_lidar_decode_ranges() -> None:
    sim = MagicMock()
    lidar = GazeboLidar(simulator=sim, own_simulator=False)
    msg = MagicMock()
    msg.ranges = [1.0, 2.0, 3.0, 5.0]
    msg.angle_min = -1.57
    msg.angle_step = 1.047
    msg.header.stamp.sec = 7
    msg.header.stamp.nsec = 0

    lidar._on_scan(msg)
    ranges = lidar.get_ranges()

    assert ranges is not None
    assert len(ranges) == 4
    assert ranges[0] == pytest.approx(1.0)
    assert ranges[3] == pytest.approx(5.0)


def test_gazebo_lidar_point_cloud() -> None:
    sim = MagicMock()
    lidar = GazeboLidar(simulator=sim, own_simulator=False)
    msg = MagicMock()
    msg.ranges = [1.0, 2.0]
    msg.angle_min = 0.0
    msg.angle_step = 1.5707963
    msg.header.stamp.sec = 0
    msg.header.stamp.nsec = 0

    lidar._on_scan(msg)
    cloud = lidar.get_point_cloud()

    assert cloud is not None
    assert cloud.shape == (2, 3)
    assert cloud[0, 0] == pytest.approx(1.0, abs=0.01)  # cos(0) * 1
    assert cloud[0, 1] == pytest.approx(0.0, abs=0.01)  # sin(0) * 1
    assert cloud[1, 0] == pytest.approx(0.0, abs=0.01)  # cos(pi/2) * 2
    assert cloud[1, 1] == pytest.approx(2.0, abs=0.01)  # sin(pi/2) * 2


def test_gazebo_lidar_no_scan_returns_none() -> None:
    sim = MagicMock()
    lidar = GazeboLidar(simulator=sim, own_simulator=False)
    assert lidar.get_ranges() is None
    assert lidar.get_point_cloud() is None
    assert lidar.get_latest() is None


def test_gazebo_lidar_get_latest_returns_reading() -> None:
    sim = MagicMock()
    lidar = GazeboLidar(simulator=sim, own_simulator=False)
    msg = MagicMock()
    msg.ranges = [1.0]
    msg.angle_min = 0.0
    msg.angle_step = 0.1
    msg.header.stamp.sec = 3
    msg.header.stamp.nsec = 0

    lidar._on_scan(msg)
    reading = lidar.get_latest()

    assert reading is not None
    assert reading.sensor_name == "front_lidar"
    assert reading.timestamp == pytest.approx(3.0)


# --- ranges_to_point_cloud standalone -----------------------------------------


def test_ranges_to_point_cloud_filters_invalid() -> None:
    ranges = np.array([1.0, float("inf"), -1.0, 2.0])
    cloud = ranges_to_point_cloud(ranges, angle_min=0.0, angle_step=0.5)
    assert cloud.shape == (2, 3)  # only 2 valid ranges


def test_ranges_to_point_cloud_empty() -> None:
    ranges = np.array([])
    cloud = ranges_to_point_cloud(ranges, angle_min=0.0, angle_step=0.1)
    assert cloud.shape == (0, 3)


# --- YOLODetector (mocked) ---------------------------------------------------


def test_yolo_detector_returns_detections() -> None:
    mock_box = MagicMock()
    mock_box.conf = [0.9]
    mock_box.cls = [2]
    mock_box.xyxy = [[10, 20, 100, 200]]

    mock_result = MagicMock()
    mock_result.boxes = [mock_box]
    mock_result.names = {2: "car"}

    with patch.dict("sys.modules", {"ultralytics": MagicMock()}):
        import importlib
        import perception.object_detector as od_mod

        mock_yolo_cls = MagicMock()
        mock_model = MagicMock()
        mock_model.return_value = [mock_result]
        mock_yolo_cls.return_value = mock_model

        with patch.object(od_mod, "YOLO", mock_yolo_cls, create=True):
            detector = YOLODetector.__new__(YOLODetector)
            detector._model = mock_model
            detector._confidence_threshold = 0.5

        image = np.zeros((480, 640, 3), dtype=np.uint8)
        detections = detector.detect(image)

    assert len(detections) == 1
    assert detections[0].class_name == "car"
    assert detections[0].confidence == pytest.approx(0.9)
    assert detections[0].bbox == (10, 20, 100, 200)


def test_yolo_detector_filters_low_confidence() -> None:
    mock_box = MagicMock()
    mock_box.conf = [0.3]
    mock_box.cls = [0]
    mock_box.xyxy = [[0, 0, 10, 10]]

    mock_result = MagicMock()
    mock_result.boxes = [mock_box]
    mock_result.names = {0: "person"}

    mock_model = MagicMock()
    mock_model.return_value = [mock_result]

    detector = YOLODetector.__new__(YOLODetector)
    detector._model = mock_model
    detector._confidence_threshold = 0.5

    detections = detector.detect(np.zeros((100, 100, 3), dtype=np.uint8))

    assert len(detections) == 0
