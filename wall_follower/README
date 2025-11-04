# Wall Follower with Computer Vision

## Quick Start

**Terminal 1:**
```bash
ros2 launch turtlebot3_cartographer cartographer.launch.py
```

**Terminal 2:**
```bash
ros2 launch wall_follower wall_follower.launch.py
```

**Test vision only:**
```bash
ros2 run wall_follower see_marker.py
```

---

## Key Modifications

### `see_marker.py`

**1. Topic Type (Lines 58-62)**
- Simulator: `Image`
- Real robot: `CompressedImage`

**2. HSV Color Thresholds (Lines 201-206)**
Adjust based on real markers using mouse callback.

**3. Camera Resolution (Lines 264, 306)**
```python
# Simulator (640×480):
centre = 320
angle = (centre - cx) * field_of_view_h / 640

# Real robot (160×120):
centre = 80
angle = (centre - cx) * field_of_view_h / 160
```

**4. Distance Calculation**
- Approach 1: `distance = distance_numerator / h` (adjust `pixel_size`)
- Approach 2: `distance = 35.772 * pow(h, -0.859)` (experimental)

### `landmark.py`

**Position Update Method**
- Current: EMA (Exponential Moving Average)
- Original: Direct replacement (commented)

---

## Calibration

Enable in `see_marker.py`:
```python
CALIBRATION_MODE = True
CALIBRATION_DISTANCE = 1000.0  # mm
```
Place marker at specified distance, run node, update `pixel_size` with output value.