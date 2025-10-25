># **Mechanical Design**

This report summarizes the design, calculations, and component selection for the mechanical design.  
The structure and actuation system are designed for a robotic platform using **three MG996R servo motors** and **two GA25-370 DC motors**.  
The main chassis is constructed from **4.5 mm thick wood and acrylic** to provide rigidity while minimizing weight.

---

## 1. System Overview

The platform consists of a dual-level chassis supporting the drive motors, gripper mechanism, and electronic components.  
The **MG996R servos** are used for the gripper and vertical motion, while the **GA25-370 DC motors** drive the rear wheels.  
The design ensures stability, sufficient torque output, and proper weight distribution.
 
## 2. Gripper Force Calculations

The torque required to lift an object is calculated as follows:

\[
T = F . r
\]

\[
F = m . g
\]

Given:

- \( m = 0.5 kg )
- \( g = 9.8 m/s^2 )

\[
F = 0.5 x 9.8 = 4.9 N
\]

\[
r = 0.1 m
\]

\[
T = 4.9 x 0.1 = 0.49 N·m
\]

Each **MG996R servo** provides approximately **5 kg·cm (0.49 N·m)** of torque.  
Since **two servos** are used in the gripper mechanism, the total available torque is:

\[
T_{total} = 2 x 5 = 10  kg·cm = 0.98 N·m
\]

Thus, the gripper design provides sufficient torque to handle an object weighing **0.5 kg**  
at a **0.1 m lever arm** with a **safety factor of about 2**.


## 3. Drive System

The drive mechanism employs **two GA25-370 DC motors** connected to the rear wheels.  
Each motor delivers adequate torque for motion and stability.

**Assumed parameters:**

- Wheel diameter = 65 mm (radius = 0.0325 m)
- Maximum linear speed = 1 m/s

\[
omega = v . r = 1 x 0.0325 = 30.77 rad/s
\]

\[
text{RPM} = {omega x 60}/{2 x pi x r} = 294 RPM
\]

Each **GA25-370 motor** can produce approximately **0.04 N·m** of torque,  
giving a total driving torque of **0.08 N·m**, sufficient for the lightweight chassis and moderate speeds.

---

## 4. Conclusion

The mechanical sub-team successfully designed a stable and functional robotic platform.  
The choice of materials, motor configuration, and torque calculations ensure reliable operation  
for both gripping and mobility tasks.

The **4.5 mm chassis** provides the necessary structural integrity while maintaining low weight for efficiency.

># **Hardware Design**

## Power Board

The Power Board was designed with safety as the top priority. The 12 V system incorporates two layers of reverse-polarity protection: first, the use of an **XL30 connector**, which physically prevents incorrect battery connection; and second, a **diode-fuse protection stage**, ensuring that even if a reverse connection occurs, the circuit remains protected.

The board also provides **regulated 5 V and 3.3 V power distribution**, each safeguarded by an **over-voltage protection circuit** implemented with **PNP transistors**. This ensures that in the event of a buck converter malfunction, the main control boards and peripheral components remain protected from potential damage.

Additionally, the Power Board includes **multiple output headers** to accommodate the connection of any additional or unplanned components without requiring hardware modifications.

## Main Control Board

The Main Control Board is built around an **ESP32 (CH340 variant)**, a capable and versatile microcontroller responsible for handling all processing tasks required by the Software team. It interfaces with various **sensor outputs**, enabling reliable data acquisition and communication necessary to perform the system's tasks effectively.

The board was also designed with **motor control flexibility**, supporting both **servo** and **stepper motors** to meet the diverse requirements of the Mechanical team.

## Buck Converter

From the very beginning, we were determined to design our own **buck converters**, as it would be a valuable contribution to the team's overall performance in the competition. The well-known **LM2596** chip was selected as the core of the Buck Board due to its proven reliability, low cost, and wide availability.

A **potentiometer** was implemented to allow precise adjustment of the output voltage, which is essential when working with **voltage-sensitive components**. Unfortunately, suitable fixed resistors for the target voltage range were not commercially available.

Special consideration was given to ensuring that the potentiometer is **sufficiently resistant to accidental adjustment**, minimizing the risk of unintentional voltage changes. Although the Power Board includes **over-voltage protection**, we prioritized **redundant safety measures** throughout the design.

## Motor Driver

Due to internal technical challenges and time constraints, designing a motor driver from scratch was not feasible. Instead, the **TB6612FNG motor driver module** was selected for its efficiency, versatility, and availability. The standalone **TB6612FNG IC** was not easily obtainable on the market, making the use of the complete module a practical choice.

The driver utilizes **MOSFETs** instead of traditional bipolar transistors, resulting in **higher efficiency**, **lower heat generation**, and the capability to **drive larger motors** reliably without thermal issues.

># **Firmware Design**

This document combines **all four modules** of the ESP32 robotic system:

1. **Encoder & Positioning System**
2. **Servo Control**
3. **MQTT Communication**
4. **Motor Driver**

---

# 1️⃣ Encoder & Positioning System

## Encoder Class
### Purpose
Handles setup and counting for one quadrature encoder using the ESP32 PCNT (Pulse Counter) hardware.

### Constructor
```cpp
Encoder::Encoder(const gpio_num_t pin_a, const gpio_num_t pin_b)
```
#### Steps
- Configure two PCNT channels for pin_a and pin_b.
- Set count limits (`INT32_MIN` to `INT32_MAX`).
- Define edge actions:
  - One channel increases count on forward movement.
  - The other decreases count on reverse movement.
- Use `ESP_ERROR_CHECK` for safety.

### Purpose
Allows accurate tracking of encoder rotations in both directions.

---

## Positioning Class

Combines **two encoders** + **MPU6050 sensor** to estimate the robot's motion.

### Constructor
```cpp
Positioning::Positioning(gpio_num_t encoder_0_pin_a, gpio_num_t encoder_0_pin_b,
                         gpio_num_t encoder_1_pin_a, gpio_num_t encoder_1_pin_b)
```

#### What it does
- Initializes two encoders.
- Starts I2C communication with the MPU6050.
- Initializes and calibrates the IMU.
- Enables the DMP (Digital Motion Processor) for filtered data.

#### Calibration Steps
```cpp
mpu.CalibrateAccel(6);
mpu.CalibrateGyro(6);
mpu.PrintActiveOffsets();
```

#### Purpose
Ensures stable sensor readings for motion tracking.

---

## Update Function

```cpp
void Positioning::update()
```
Called repeatedly to update the robot's position.

### Steps
1. **Read IMU Data**
   - Extracts quaternion, gravity, and yaw/pitch/roll.
2. **Compute Wheel Movement**
   ```cpp
   dl = encoder0.count() * (2π * WHEEL_RADIUS) / ENCODER_RESOLUTION;
   dr = encoder1.count() * (2π * WHEEL_RADIUS) / ENCODER_RESOLUTION;
   ```
3. **Reset Encoders**
   ```cpp
   encoder.reset();
   ```
4. **Calculate Position Change**
   ```cpp
   d = (dl + dr) / 2.0;
   x += d * cos(yaw);
   y += d * sin(yaw);
   ```

---

## Mathematical Model
| Symbol | Meaning | Formula |
|---------|----------|----------|
| `dl, dr` | Left/Right wheel distance | `(encoder_count * 2π * R) / resolution` |
| `d` | Average displacement | `(dl + dr)/2` |
| `θ` | Orientation (yaw) | From MPU6050 |
| `x, y` | New position | `x += d * cos(θ)` , `y += d * sin(θ)` |

### Summary
| Feature | Description |
|----------|--------------|
| Encoders | Track linear movement |
| IMU | Provides orientation (yaw) |
| Fusion | Combines encoder + IMU data |
| Calibration | Performed automatically |
| Future Work | Add Kalman Filter |

### Notes
- Encoders may need pull-up resistors.
- MPU6050 connects via SDA/SCL.
- Yaw angle in radians.
- Adjust wheel radius & resolution for your hardware.

### Dependencies
- `esp32-hal.h`
- `Wire.h`
- `driver/pulse_cnt.h`
- `MPU6050.h`

---

# 2️⃣ Servo Control with ESP32

## Overview
Reads a PWM input signal (e.g., from a receiver) and converts it into a corresponding servo output angle using ESP32’s LEDC PWM module.

## Hardware Setup
| Component | Pin | Direction | Description |
|------------|-----|-----------|--------------|
| Servo Signal | GPIO 18 (`Spin`) | OUTPUT | Sends PWM signal to servo |
| Control Input | GPIO 12 (`Cpin`) | INPUT | Reads PWM input from receiver |

---

## Code Steps

### 1. Initialization
```cpp
Serial.begin(115200);
pinMode(Spin, OUTPUT);
pinMode(Cpin, INPUT);
```
Sets up pins and serial monitor.

### 2. PWM Setup
```cpp
ledcSetup(0, 50, 16);
ledcAttachPin(Spin, 0);
```
- 50 Hz → Standard for servos  
- 16-bit resolution (0–65535)

### 3. Read Input Signal
```cpp
int val = pulseIn(Cpin, HIGH, 25000);
```

### 4. Mapping to Servo Angle
```cpp
int angle = map(val, 500, 1500, 0, 180);
int servoPulse = map(angle, 0, 180, 500, 2500);
```

### 5. Output PWM
```cpp
int duty = (servoPulse * 65535) / 20000;
ledcWrite(0, duty);
```

### 6. Debug Output
```cpp
Serial.print("Angle: ");
Serial.println(angle);
```

---

## Summary
| Step | Description |
|------|--------------|
| 1 | Read input PWM |
| 2 | Convert to servo angle |
| 3 | Generate PWM for servo |
| 4 | Output and debug |

**Notes:**
- Ensure common ground for ESP32, receiver, and servo.
- Adjust pulse ranges for your servo model.

---

# 3️⃣ ESP32 MQTT Communication Module

## Overview
Connects ESP32 to Wi-Fi and an MQTT broker for message exchange between robot parts (motor, arm, gripper, location). Uses FreeRTOS queues.

## Included Libraries
```cpp
#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
```

### Functions
#### `callback()`
Handles incoming MQTT messages → sends to proper queue.

#### `setupWiFi()`
Starts Wi-Fi access point and prints IP.

#### `reconnect()`
Keeps MQTT connection alive.

#### `send_data()`
Main FreeRTOS task:
- Reads queues and executes commands.
- Publishes location data back via `send_robotlocation`.

#### `setup()`
Initializes queues and creates the FreeRTOS task.

---

## MQTT Topics
| Topic | Direction | Description |
|--------|------------|-------------|
| `movement_body` | Subscribe | Motor commands |
| `movement_arm` | Subscribe | Arm control |
| `movement_gripper` | Subscribe | Gripper control |
| `robot_location` | Subscribe | Location data |
| `send_robotlocation` | Publish | Robot location response |

**Author:** Kenzy Ibrahim  
**Protocol:** MQTT over Wi-Fi  
**RTOS:** FreeRTOS Queues  

---

# 4️⃣ Motor Driver Module (ESP32)

## Overview
Controls left & right DC motors via PWM and direction pins. Converts motion commands into wheel velocities using PID.

## Hardware Setup
| Component | Pin | Description |
|------------|-----|--------------|
| L_IN1 | 25 | Left motor direction 1 |
| L_IN2 | 26 | Left motor direction 2 |
| L_PWM | 27 | Left motor PWM |
| R_IN1 | 32 | Right motor direction 1 |
| R_IN2 | 33 | Right motor direction 2 |
| R_PWM | 12 | Right motor PWM |

---

## Constants
| Constant | Meaning |
|-----------|----------|
| `WHEEL_BASE` | Distance between wheels (0.2 m) |
| `wheel_radius` | 0.03 m |
| `MAX_WHEEL_SPEED` | 30 × radius |
| `kp`, `ki`, `kd` | PID gains |
| `dt_ms` | Control interval (50 ms) |

---

## Main Functions

### `MotorInit()`
Initializes pins and PWM channels.

### `setMotor()`
Sets direction and PWM duty cycle.

### `linearSpeedtoPWM()`
Converts wheel linear speed → PWM.

### `moveRobot()`
Calculates wheel speeds and drives motors.

### `computePID()`
PID controller for angular correction.

### `timingfunc()`
Maintains control loop timing.

---

## System Flow Diagram
```
Motion Command → moveRobot() → Left/Right Wheel Speed
 → linearSpeedtoPWM() → setMotor() → Motor Movement
```

---

## Integration with RTOS
Motor commands are sent via queue structure:
```cpp
typedef struct {
  float angle;
  float magnitude;
} MotorCommand_t;
```

Handled by a `MotorTask()` that executes `moveRobot()` periodically.

---

**End of Combined Documentation**


># **Software Design**
## Layout
```plaintext
Software/
└── 📁 RobotGui/   
    ├── 📁 core/
    │   ├── 📁 auto/                
    │   |   └── under construction ...  
    |   ├── 📁 communication/ 
    |   |   ├── 📁 publish/
    |   |   |   └── 📄 movement.py 
    |   |   ├── 📁 subscribe/
    |   |   |   ├── 📄 coordinates.py
    |   |   |   ├── 📄 arm_position.py 
    |   |   |   ├── 📄 gripper_state.py 
    |   |   |   └── 📄 Subscribers_methods.py
    |   |   └── 📄 client.py
    |   ├── 📁 control/
    |   |   ├── 📄 controller.py
    |   |   ├── 📄 keyboard_controls.py
    |   |   ├── 📄 modes.py
    |   |   └── 📄 robot_controller,py
    |   └── 📁 cv/                      
    |       ├── 📄 color_detection_and_recognition.py
    |       ├── 📄 cv.py
    |       ├── 📄 detections.py
    |       ├── 📄 main_cv.py
    |       └── 📄 QR_scanner.py
    |              
    ├── 📁 gui/
    │   ├── 📄 camera_display.py      
    │   ├── 📄 minimap.py    
    |   ├── 📄 QR_display.py      
    │   ├── 📄 settings.py  
    │   └── 📄 window.py 
    |
    ├── 📁 placeholders/
    │    
    └── 📄 __main__.py
```

## Communication Module

### Overview

This system provides MQTT-based communication for a robot control GUI, handling both publishing commands and subscribing to robot state updates. The architecture follows a modular design with separate components for different robot subsystems.

### ✨ Layout Structure:

```plaintext
📁 communication/ 
├── 📁 publish/
|   └── 📄 movement.py 
├── 📁 subscribe/
|   ├── 📄 coordinates.py
|   ├── 📄 arm_position.py 
|   ├── 📄 gripper_state.py 
|   └── 📄 Subscribers_methods.py
└── 📄 client.py       

```
---
### 1. MQTT Client (`client.py`)

The main MQTT communication handler that manages both publishing and subscribing capabilities. Contains setup logic for communications

#### Key Features:
- **Dual functionality**: Handles both message publishing and subscription setup
- **Thread-safe publishing**: Uses background threads for non-blocking message delivery
- **Modular subscription**: Separate setup methods for each topic subscribed to
- **publishing**: Publishing setups once for all topics

#### Main Classes:

##### `Mqtt` Class
- **Initialization**: Connects to MQTT broker (default: localhost:1883 -> mosquitto is used in the device)

- **Publishing Setup**: Creates publisher client

- **Subscription Setup Methods for each topic**:
  - `setup_coordinates()`: For robot position coordinates
  - `setup_arm_position()`: For arm joint positions  
  - `setup_gripper_state()`: For gripper open/close state

- **Publishing Methods**:
    - `publishing_setup()`: Initializes MQTT publisher client
    - `publish_msg()`: Public method to publish messages (threaded)
    - `_publishing()`: Internal threaded publishing implementation

### 2. Movement Publishing (`publish\movement.py`)

Handles command publishing for robot movement control.

#### `Movement_Publish` Class:
- **Body Movement**: Publishes `movement_body` messages with magnitude/angle or x/y coordinates
- **Arm Control**: Publishes `movement_arm` messages with direction (-1, 0, 1)
- **Gripper Control**: Publishes `movement_gripper` messages with open/close state (0/1)

### 3. Subscription Handlers (in `subscribe\`)

- coordinates.py (**Coordinates** Class)
  - Subscribes to `robot_coordinates` topic
  - Parses comma-separated x,y,theta coordinates and oreintation
  - Calls `slot` function with parsed float values

- arm_position.py (**Arm_Position** Class) 
  - Subscribes to `arm_position` topic
  - Handles arm joint angle data
  - Calls `slot` function with `theta` value

- gripper_state.py (**Gripper_State** Class)
  - Subscribes to `gripper_state` topic
  - Handles gripper state (open/closed)
  - Calls `slot` function with `state` integer

> `slot` function is where the data after being decoded will go to the function that will use the data   
in the setup of each subscriber's topic a slot is saved to carry the data and a function in assigned to use this data

### 4. GUI Integration (`subscribe\Subscribers_methods.py`)

QT-based widget that uses `slot` data for updating UI with robot data.

#### **SubscribersMethods** Class:
- **`update_coordinates(x, y)`**: Updates coordinate display label
- **`arm_position(theta)`**: Updates arm position display
- **`gripper_state(state)`**: Updates gripper state display
- Uses QT (gui library used) slots for thread-safe UI updates (forces function to obey the QT main thread)
---
### ✨ Message Topics

#### Publishing Topics:
- `movement_body`: Robot base movement commands
- `movement_arm`: Arm movement commands  
- `movement_gripper`: Gripper control commands

#### Subscription Topics:
- `robot_coordinates`: Robot position updates
- `arm_position`: Arm joint position updates
- `gripper_state`: Gripper state updates

### ✨ Data Formats

#### Movement Commands:
- **Body**: `"magnitude,theta"` or `"x,y"`
  - x,y coordinates (float, *range:* [0,3.20])
  - magnitude (float, *range:* [0,1])
  - theta (int, *range:* [0,360])
- **Arm**: `"direction"` (-1=down, 0=nothing, 1=up)
- **Gripper**: `"state"` (0=closed, 1=open)

#### Subscription Data:
- **Coordinates**: `"x,y,theta"` (comma-separated floats)
  - x,y coordinates (float, *range:* [0,3.20])
  - theta (int, *range:* [0,360])
- **Arm Position**: `"value"` (represents arm position)
- **Gripper State**: `"state"` (integer: open/closed)

### ✨ Simple Usage Example

```python
# Initialize MQTT client
mqtt_client = Mqtt('localhost', 1883)

# Setup subscriptions
mqtt_client.setup_coordinates(update_coordinates_slot)
mqtt_client.setup_arm_position(update_arm_slot)

# Initialize movement publisher
movement = Movement_Publish(mqtt_client)

# Send movement commands
movement.publish_body_movement(1.0, 45.0)  # Move with magnitude 1 at 45°
movement.publish_arm_movement(1)           # Move arm positive direction
movement.publish_gripper_state(1)          # Open gripper
```

### ✨ Architecture Notes

- **Separation of Concerns**: Publishing and subscribing are handled by separate classes
- **Thread Safety**: Publishing uses background threads, UI updates use QT slots
- **Extensibility**: Easy to add new message types and topics
- **Error Handling**: Basic connection failure handling with retry mechanisms

### ✨ Dependencies

- `paho-mqtt`: MQTT client library
- `PySide6`: QT framework for GUI
- `threading`
   
---

## Control Module

### Layout
```
📁 control/ 
├── 📄 controller.py            # control via controller
├── 📄 keyboard_controls.py     # control via keyboard
├── 📄 modes.py                 # enum file
└── 📄 robot_controller.py      # main manager of controls
``` 
---
---

###  Keyboard Controls Module (`keyboard_controls`)

It listens to which keys are pressed and converts them into **commands** that control the robot’s **movement**, **arm**, and **gripper**.

---



#### flow of what happens:

So, when you press a key:

1. The GUI detects which key was pressed.
2. **Keyboard_Command class** decides what kind of action that key means (e.g., move forward, lift arm).
3. It builds a command and sends it to the **RobotController class** in `robot_controller.py`.

---

#### 🎮 The Control System

The keyboard is mapped to different robot actions:

 1. Movement Control

You can use the **WASD keys** to move the robot in any direction:

| Keys | Movement | Angle (degrees) |
|------|-----------|----------------|
| W | Forward | 0° |
| S | Backward | 180° |
| A | Left | 270° |
| D | Right | 90° |
| W + D | Forward-Right | 45° |
| W + A | Forward-Left | 315° |
| S + D | Backward-Right | 135° |
| S + A | Backward-Left | 225° |

If no movement key is pressed, a “stop” signal is sent, telling the robot to stay still.

---

 2. Arm Control

The robot’s arm can be moved **up or down** using keyboard keys:

| Key | Action |
|-----|---------|
| Y | Move Arm Up |
| N | Move Arm Down |

If no movement key is pressed, a “stop” signal is sent, telling the arm to stay still.

---

### 3. Gripper Control

The robot has a gripper that can **open** and **close** to pick or release objects.

| Key | Action |
|-----|---------|
| Spacebar | Toggle between Open and Close |

Each time you press space, it switches the gripper state:
- If it was open → it closes.
- If it was closed → it opens.


---
---

### Joystick Controller Module

#### Control Mapping
- Analog Sticks

    - Left Stick: Movement control (Axis 0-1)

    - Output : Polar coordinates (magnitude 0-1, angle 0-360°)

    - Deadzone: 0.1 threshold for noise reduction

- Button Controls

            |  Button        |   Function    | 
            |----------------|---------------|
            |      X (0)     | Arm Down      | 
            |  Triangle (3)  | Arm	Up       | 
            |----------------| --------------|
            |  Circle (1)    | Gripper Open  |
            |  Square (2)    | Gripper Close |
            |----------------|---------------|
            |   R1 (5)       | QR scanning   |
            |----------------|---------------|

#### Control Logic
- Movement Processing

    - Cartesian to polar coordinate conversion

    - Angle normalization (0-360° range)

    - Safety: zero magnitude = zero angle

- Event Handling

    - Button press: Immediate action execution

    - Button release: Reset to neutral state

    - 10ms polling interval for smooth operation

#### Data Flow

- Joystick Input → Pygame Events → Polar Conversion → Robot Commands

#### Output Format

```python
[magnitude, angle, arm_state, gripper_state]
```
- magnitude: Movement intensity (0-1)

- angle: Direction in degrees (0-360)

- arm_state: -1(down), 0(neutral), 1(up)

- gripper_state: 0(closed), 1(open)

**RobotController class** in `robot_controller.py`.

---
---

### `modes.py

``` python
from enum import Enum

class Mode(Enum):
    Manual=1
    Semi_Auto=2
    Full_Auto=3

```

---
---

### **RobotController Class** in `robot_control.py`

```python
class RobotController():
```
Controls the robot’s movement and mode (manual or full auto).

#### Important Logic

- **Mode switching:**
  ```python
  def setMode(self, new_mode: Mode):
      print(f'switching to {new_mode.name}')
      self.mode = new_mode
  ```

- **Manual control:**
  ```python
  self._move.publish_body_movement(cmd[0], cmd[1])
  self._move.publish_arm_movement(cmd[2])
  self._move.publish_gripper_state(cmd[3])
  ```
  Publishes movement commands to the robot using MQTT.
  > commands are received from keyboard or controller

---
---

## CV Module

This document explains the core logic and purpose of the major components in the robot’s GUI and vision system, focusing on important lines and tracker logic.

---

### 1. `color_detector` Function -->RobotGui/core/cv/detections

```python
def color_detector(frame: cv2.Mat, ranges: list, initialized: bool, tracker: cv2.TrackerCSRT, zone: bool):
```
- Detects a colored object and uses OpenCV's CSRT tracker to follow it.
- `ranges` contains HSV color ranges to detect specific colors.
- `initialized` determines if the tracker is currently active.

#### Key Logic
- **Mask creation:**
  ```python
  blank_mask = np.zeros(frame.shape[:2], dtype=np.uint8)
  blank_mask[:,20:frame.shape[1]-20] = 255
  frame = cv2.bitwise_and(frame, frame, mask=blank_mask)
  ```
  Ensures that the detected object is centered in the x axis

- **Tracker update:**
  ```python
  ok, bbox = tracker.update(frame)
  if ok:
      x, y, w, h = [int(v) for v in bbox]
      cv2.rectangle(frame, (x, y), (x+w, y+h), (0,255,0), 2)
  ```
  If tracking is successful, the bounding box (ROI) is drawn around the detected object.

- **Lost tracking:**
  ```python
  else:
      initialized = False
  ```
  Stops tracking when the object is lost.

- **Object detection when tracker not initialized:**
  Converts frame to HSV, applies color masks, and finds contours to locate the largest detected color area.

- **Tracker initialization:**
  ```python
  tracker.init(frame, (x1, y1, w1, h1))
  initialized = True
  ```
  Starts tracking based on the detected color region.

---

### 2. `Camera` Class -->RobotGui/core/cv/cv.py

```python
class Camera:
```
Handles continuous camera feed, QR code scanning, and color detection in separate threads.

#### Important Parts

- **Threaded frame reading:**
  ```python
  self._frame_thread = Thread(target=self._frame_loop, daemon=True)
  self._frame_thread.start()
  ```
  Continuously reads camera frames without blocking the main program.

- **Frame loop:**
  ```python
  success, self.image = self._cap.read()
  ```
  Captures frames in real-time; restarts capture if connection fails.

- **Color detection thread:**
  ```python
  detector = ColorDetection(img, 'green')
  detected = detector.DetectColor()
  ```
  Continuously detects a chosen color (`green` here) and prints its position.

- **QR code thread:**
  ```python
  result = qr_scanner(img)
  ```
  Starts scanning for QR codes in a parallel thread which starts manually.

---

### 3. `qr_scanner` Function -->RobotGui/core/cv/QR_scanner

```python
def qr_scanner(img):
```
Uses OpenCV’s built-in QRCodeDetector to find and decode QR codes.

#### Key Lines
- **Detection and decoding:**
  ```python
  coords, bbox, straight_qrcode = scan.detectAndDecode(img)
  ```
  Returns decoded data and bounding box.

- **Error handling:**
  ```python
  if bbox is None or coords == "":
      return 'no QR code'
  ```
  Prevents crashes when no QR code is found.

---



### 5. `Minimap` Class -->RobotGui/gui/minimap

```python
class Minimap(QWidget):
```

#### Important Lines

- **Scene and graphics setup:**
  ```python
  self._scene = QGraphicsScene(self)
  self._robot = QGraphicsEllipseItem()
  self._background = QGraphicsRectItem()
  ```
  Creates the minimap background and robot marker.

- **Coordinate updates:**
  ```python
  x, y = self._subscriber.coordinates
  xmap = x * self._square_size / 3
  ymap = y * self._square_size / 3
  self._robot.setPos(xmap, ymap)
  ```
  Updates robot’s position on the minimap according to real data.

---

### Tracking Properties
- The class also contains some interesting properties that will help in trcking the object :
    - ***saturation*** : returns the coordinates of the center of the area at which the color is saturated 
    - ***sat_dist_to_center*** : returns the difference between the ***saturation*** point and frame center
    - ***right_position*** : returns a *str* that indicates whether the color in the frame center or not
    - ***frame_sat_ratio*** : returns the ratio between the frame area and the color area that when approaches one means that you are so close to the color so the robots stops *(in auto mode)*


### Color Recognition Module
- **RecognizeColor** is the function that state the color it recognizes in the central pixel of the frame through looping over all the given colors returning the name of the color as a *str*


### Core Components
#### HSV_LowerUpper(BGRcolor)
- Purpose: Converts BGR color to HSV range for masking

- Input: BGR color tuple

- Output: Lower/upper HSV bounds with hue wrap-around for red

#### FormMask(image, color)
- Purpose:
    - Creates binary mask using HSV thresholding
    - Applies noise reduction (Otsu + morphological operations)
```python
 _, mask = cv2.threshold(mask , 0 , 255 , cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    mask = cv2.morphologyEx(mask,cv2.MORPH_CLOSE,np.ones((3,3),np.uint8))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3,3),np.uint8))
```
- Input: frame and the color we would like to mask

- Output: a mask with low nose that helps in color detection

#### Color Dictionary
```python
COLORS_BGR = {
    'red': [20, 20, 230], 'green': [20, 230, 20],
    'blue': [230, 20, 20], 'yellow': [0, 230, 230],
    'orange': [0, 165, 230], 'purple': [128, 0, 128],
    'pink': [203, 192, 230], 'brown': [42, 42, 165]
}
```
---



### Summary

This system integrates:
- **Computer Vision (OpenCV)** for color detection and QR tracking.  
- **Multithreading** for parallel camera and detection processes.  
- **GUI (PySide6)** for visualization.  
- **MQTT Communication** for robot control commands.

Each part contributes to an autonomous or semi-autonomous or manual robot interface combining perception, control, and visualization.


## GUI Module

### Graphical User Interface (GUI) Documentation

#### Overview

The Robot GUI is a desktop application built using the **PySide6** (Qt) framework. Its primary function is to serve as the **Human-Robot Interface (HRI)**, providing the pilot with a camera feed, a real-time minimap of the robot's location, critical status information, and control selection capabilities, as required by the rulebook's **Console** section.

#### ✨ Layout Structure:

```plaintext
📁 gui/ 
├── 📄 window.py        # Main application window and structure
├── 📄 camera_display.py # Displays video feed from robot camera
├── 📄 minimap.py      # Displays robot position and playground map
├── 📄 info_display.py # Displays QR data and robot states
└── 📄 settings.py     # Handles mode selection (Manual/Semi-Auto/Full-Auto)
```

-----

#### 1\. Main Application Window (`window.py`)

The `Window` class is the main `QMainWindow` that sets up the overall application structure, window resizing logic, and global event handlers.

### Main Classes:

##### `Window` Class

  - **Layout Management**: Uses a custom `CentralWidget` to hold the main components. It enforces an **11:6 aspect ratio** for the main content area to ensure visual consistency, adjusting to available window space.
  - **Keyboard Control**: Overrides `keyPressEvent` and `keyReleaseEvent` to delegate control inputs to the `settings.keyboard_instructions` module.
  - **Communication**: Initializes the global `Mqtt` client (`_mqtt`) and the `Movement_Publish` system.
  - **Timer Start**: Connects the **"Start Timer" button** on the console to the `StatusBar` countdown.

##### `CentralWidget` Class

  - The main container for the GUI. Uses an `QHBoxLayout` to divide the interface into two main columns: the **Camera Display** (stretch 8) and the **Widget Grid** (stretch 3), prioritizing the video feed size.

##### `WidgetGrid` Class

  - Organizes the right-hand column using a `QVBoxLayout`, stacking the:
    1.  **Minimap** (`Minimap`) - *Expanding size policy*
    2.  **Information Display** (`InfoDisplay`) - *Fixed size policy*
    3.  **Settings Dropdown** (`Settings`) - *Fixed size policy*
    4.  **Start Timer Button** (`QPushButton`) - *Fixed size policy*
  - Also responsible for setting up the MQTT subscription for coordinates, linking the data flow to the `Minimap`'s update method.

##### `StatusBar` Class

  - Displays critical information at the bottom of the window.
  - **Timer/Countdown**: Implements a maximum **10-minute (600 seconds)** round timer (`_seconds`), which decrements when the `_countdown_flag` is set via the **Start Timer** button.
  - **Robot Status**: Shows the remaining time and the **Robot Connected/Disconnected** status.

-----

### 2\. Camera Display (`camera_display.py`)

Handles the live video stream from the robot's camera using computer vision libraries.

#### `CameraDisplay` Class:

  - **Video Source**: Instantiates a `Camera` object (from `RobotGui.core.cv.cv`) to capture frames.
  - **Display**: Uses a `QLabel` (`_frame_view`) to show the video feed.
  - **Frame Rate**: Uses a **QTimer** set to **17ms (approx. 60 FPS)** to poll the `Camera` device for new frames and update the display via `update_view()`.
  - **Aspect Ratio**: Enforces a **4:3 aspect ratio** for the video display area.
  - **Conversion**: Converts the `cv2` (BGR) frame data into a **PySide6 QPixmap** using `QImage.Format_BGR888` for display.

-----

### 3\. Information Display (`info_display.py`)

Displays key non-visual data points from the robot, fulfilling the requirement for displaying QR-decoded coordinates.

#### `InfoDisplay` Class:

  - **Layout**: Uses a `QGridLayout` to arrange status labels.
      - Top row: `_arm_label` and `_gripper_label` for arm/gripper status.
      - Bottom row: `_qr_label` spanning two columns for decoded QR data.
  - **QR Polling**: Instantiates a `Camera` object and uses a **QTimer** set to **200ms** to regularly check the `self._camera.last_qr` property, ensuring the UI is updated immediately after a QR code is successfully scanned and decoded.
  - **Styling**: Labels are color-coded (QR label is blue) for visual distinction.

-----

### 4\. Minimap (`minimap.py`)

Provides a visual representation of the robot's location on the **3m x 3m playground**.

#### `Minimap` Class:

  - **Visualization**: Uses a **QGraphicsScene** and **QGraphicsView** to draw the map.   
  Displays a real-time map of robot position using PySide6 graphics.

  - **Playground Representation**: A `QGraphicsRectItem` (`_background`) represents the 3m x 3m area.
  - **Robot Representation**: A `QGraphicsEllipseItem` (`_robot`) represents the robot's current position (red circle).
  - **Coordinate System**: The graphics view is transformed (`QTransform`) to align with standard Cartesian coordinates: **y-axis is flipped** so that the origin ($x=0, y=0$) is at the bottom-left corner, as is standard for the playground.
  - **Localization Display**: A `QLabel` (`_coords_label`) displays the robot's received coordinates and orientation (x, y, theta).
  - **Integration**: Initializes `SubscribersMethods` to connect the MQTT coordinate updates to the `_coords_label`. *The current `update_coordinates` method is a placeholder for updating the robot's visual position on the map based on the received coordinates.*

-----

### 5\. Settings and Controls (`settings.py`)

Handles the selection of operational modes and routes keyboard input to the robot's control system.

#### `Settings` Class (inherits `QComboBox`):

  - **Mode Selection**: Allows the pilot to select **'Manual'**, **'Semi-Auto'**, or **'Full-Auto'** mode as required for bonus tasks and overall operation.
  - **Mode Switching**: The `_on_mode_change` slot calls `self._robot_controller.setMode()` with the corresponding `Mode` enum (`Mode.Manual`, `Mode.Semi_Auto`, `Mode.Full_Auto`).
  - **Control System**: Initializes `RobotController` (for communication) and `Controller` (for higher-level logic).
  - **Keyboard Input**: Initializes the global `keyboard_instructions` as a **`Keyboard_Command`** object, which is then used by the main `Window` to process key presses and releases for manual control.
