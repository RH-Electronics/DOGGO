# 🐶 DOGGO Controller

**DOGGO Controller** is a firmware for controlling a quadruped robot based on **Teensy 4.0** with inverse kinematics (IK) and **LX-16A serial bus servos**.
**Work in Progress** 

## 🚀 Features
- **Inverse Kinematics (IK)** for leg control
- **Support for LSC Serial Servo Controller** (Hiwonder Bus)
- **Serial communication** (via PC or Raspberry Pi)
- **Postures and walking mode, Work still in pregress!** (stand, sit, walk, etc.)
- **Sensor integration support (future feature)**

---

## 🛠 Requirements
- **Teensy 4.0 or Teensy 4.1**
- **LX-16A Serial Bus Servos** (Hiwonder Bus)
- **LSC Servo Controller** (connected via `Serial1`) [Hiwonder Serial Bus Servo Controller](https://www.hiwonder.com/products/serial-bus-servo-controller)
- **Raspberry Pi / PC** (for sending commands via Serial)
- **DOGGO Quadruped Robot** [Original DOGGO](https://www.thingiverse.com/thing:4916566) or my remixed 3D model [DOGGO](https://www.thingiverse.com/thing:6950224)

---

## 🔧 Installation
1. **Clone the repository**:
   ```bash
   git clone https://github.com/RH-Electronics/DOGGO.git
   cd DOGGO
   ```
2. **Open the project in Arduino IDE**
3. **Connect the Teensy 4.0** and upload `DOGGO_controller.ino`
4. **Connect Raspberry Pi / PC via Serial**
5. **Start issuing commands**

---

## 🕹 Serial Command Control
Commands are sent via **Serial (115200 baud rate)**.

### 📌 Main Commands:
| Command         | Description |
|----------------|-------------|
| `WALK 16 400`  | Walk **16 cycles** at speed **400** |
| `STAND`        | Stand in the default posture |
| `STANDH`       | Stand higher |
| `SIT`          | Sit down |
| `LIEDOWN`      | Lie down |
| `GETBAT`       | Get battery voltage in mV |

Example of a command sent via **Serial Monitor**:
```
WALK 40 500
```
```
STAND 1000
```

---

## 📂 Project Structure
```
DOGGO/
├── DOGGO_controller.ino  # Main firmware file
├── IK.cpp                 # Inverse Kinematics implementation
├── IK.h                   # Header file for IK
├── LSCSerial.cpp          # LX-16A servo control implementation
├── LSCSerial.h            # Header file for servos
```

---

## 📌 Future Enhancements
- 🚀 **Integration of sensors (BNO055)**
- 🚀 **Add more commands**
- 🤖 **Autonomous mode support**
- 🔥 **AI LLM integration, control via Raspberry Pi**

👨‍💻 **Author:** [RH-Electronics](https://github.com/RH-Electronics)

Code made with [Airis](https://static.wixstatic.com/media/e43988_90b2957681f44ae6982181c0cf52bb7a~mv2.jpg).

🤖 **License:** MIT

