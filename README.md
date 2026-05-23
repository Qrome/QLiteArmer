# QLiteArmer — MSP‑Driven Auto‑Arming Module with PWM Arming & Servo Expansion

**QLiteArmer** is a compact, high‑reliability RP2040‑based module that automatically arms DJI O4 / Vista / Air Unit systems using MSP telemetry detection. It provides deterministic arming behavior, clean state‑machine architecture, optional PWM‑based arming control, and a built‑in servo expander with frequency‑matching output.

This project is designed for FPV pilots, RC builders, and embedded developers who want a robust, plug‑and‑play arming solution with professional‑grade signal handling.

---

## ✨ Features

### **Pin Map Summary**
| Function | Pin(s) | Notes |
| --- | --- | --- |
| **CRSF / ELRS UART** | GP8 (TX), GP9 (RX) | Primary RC link |
| **MSP UART (VTX)** | GP0 (TX), GP1 (RX) | DJI / Walksnail |
| **I²C (BMP280)** | GP4 (SDA), GP5 (SCL) | Barometer |
| **VBAT ADC** | GP26 | Voltage divider |
| **PWM CH1–CH8** | 13, 12, 11, 10, 7, 6, 3, 2 | Servo/motor outputs |

### **MSP‑Driven Auto‑Arming**
- Automatically detects DJI O4 / Vista / Air Unit via MSP telemetry.
- Fully non‑blocking MSP parser ensures reliable detection.
- Supports STATUS, STATUS_EX, and ANALOG messages.
- Clean LED feedback for each state.

### **Deterministic State Machine**
- **BOOT_DETECT** → Wait for MSP heartbeat  
- **PRE_ARM_DELAY** → Timer or PWM‑based arming  
- **ARMED** → Active arming state  
- **ERROR** → Timeout or invalid MSP  
- Designed for clarity, maintainability, and future expansion.

### **Dual Arming Modes**
#### **1. Timer‑Based Auto‑Arming**
- Arms automatically after a configurable delay.

#### **2. PWM‑Based Arming**
- Reads a standard RC PWM channel (1000–2000 µs).
- Arms when PWM exceeds a configurable threshold.
- Disarms when PWM drops below threshold.
- Fully non‑blocking and jitter‑resistant.

### **Servo Expander (PWM Input → Scaled PWM Output)**
- Reads PWM input on **pin 7** from any RC receiver (ELRS, FrSky, Futaba, Spektrum, etc.).
- Supports **50–500 Hz** input frequencies.
- Measures pulse width and input period in real time.
- Outputs a **scaled PWM signal** on **pin 29**:
  - Input: 1000–2000 µs  
  - Output: 500–2500 µs (configurable)
- Output frequency **matches the input frequency**.
- Uses RP2040 **hardware PWM** for rock‑solid timing.
- ISR is ultra‑lightweight and validated for ELRS PWM receivers.

### **Non‑Blocking Architecture**
- MSP parsing  
- PWM arming  
- Servo expansion  
- LED updates  
- State machine transitions  

…all run without blocking the main loop.

### **Clean Modular Codebase**
QLiteArmer is organized into clear, purpose‑built modules:

| File | Purpose |
|------|---------|
| **QLiteArmer.ino** | Main application entry point and loop integration |
| **config.h** | All user‑tunable parameters (pins, thresholds, timing) |
| **bf_msp.cpp / bf_msp.h** | MSP message definitions and helpers |
| **detection.cpp / detection.h** | MSP detection logic and heartbeat validation |
| **state_machine.cpp / state_machine.h** | Core arming state machine |
| **led.cpp / led.h** | LED status indication and color/state mapping |
| **servo_expander.cpp / servo_expander.h** | PWM input capture + frequency‑matched PWM output |

This structure makes the firmware easy to extend, debug, and maintain.

### **Robust Hardware Behavior**
- INPUT_PULLDOWN on PWM inputs prevents floating edges.
- Validates pulse width and period to reject noise.
- Safe defaults ensure predictable startup behavior.

---

## 🧩 Hardware Requirements

- RP2040‑Zero or compatible RP2040 board  
- DJI O3, O4 / Vista / Air Unit / Walksnail UART connection  
- Optional:
  - RC PWM receiver (ELRS PWM RX recommended)
  - Servo / gimbal / accessory for PWM expansion output

---

## 📡 Pin Assignments

| Function               | Pin     | Notes                          |
|------------------------|---------|--------------------------------|
| MSP RX                 | GPIO0/1 | UART from Air Unit             |
| PWM Arming Input       | GPIO6   | Optional                       |
| Servo Expander Input   | GPIO7   | From RC receiver               |
| Servo Expander Output  | GPIO29  | Scaled PWM output (pan servo)  |
| Status LED             | GPIOx   | Configurable                   |

---

## ⚙️ Configuration

All user‑adjustable settings live in `config.h`:

- MSP timeouts  
- Arming thresholds  
- PRE_ARM_DELAY duration  
- PWM input/output ranges  
- Servo expander scaling  
- Pin assignments  

---

## 🚀 How It Works

1. **Boot**  
   - LED turns blue  
   - Module waits for MSP telemetry  

2. **MSP Detected**  
   - LED turns red  
   - PRE_ARM_DELAY begins  

3. **Arming Logic**
   - LED turns green when armed  
   - Timer‑based arming  
   - OR PWM‑based arming  

5. **Servo Expansion**  
   - Continuously reads PWM input  
   - Measures pulse width + frequency  
   - Outputs scaled PWM on pin 29  
   - Fully independent of arming logic  

---

## 🧪 Tested With

- ELRS PWM receivers (50–500 Hz)  
- DJI O4 Air Unit MSP telemetry  
- Standard RC PWM receivers  
- Bench servos and gimbal controllers  
- RP2040‑Zero hardware  

---

## 📄 License

MIT License — feel free to modify, extend, and integrate into your own builds.
