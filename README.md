# QLiteArmer — FULL OSD with MSP‑Driven Arming Module with ELRS/CRSF Channel Breakout & Servo Expansion

**QLiteArmer** is a full OSD, on high‑reliability RP2040‑based module that automatically arms Walksnail Avatar HD, DJI O3/O4 HD video systems using MSP telemetry detection. It provides deterministic arming behavior, clean state‑machine architecture, PWM‑based arming control, and a built‑in servo expander.

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
| **RGB LED** | GP16 | Built in Indicator |
| **GPS** | 3v3, Gnd, GP14 (TX), GP15 (RX) | 4 Wire GPS |

### **MSP‑Driven VTx Arming**
- Automatically detects Walksnail Avatar HD & DJI O3 or O4 Air Unit via MSP telemetry.
- Fully non‑blocking MSP parser ensures reliable detection.
- Supports MSP Display port OSD by emulating Betaflight OSD elements
- RP2040 LED feedback for each state.

### **Deterministic State Machine**
- **BOOT_DETECT** → Wait for MSP heartbeat (blue)  
- **PRE_ARM_DELAY** → Ready to arm with PWM‑based arming (red) 
- **ARMED** → Active arming state (green)  
- **ERROR** → Timeout or invalid MSP (amber)  
- Designed for clarity, maintainability, and future expansion.

#### **PWM‑Based Arming**
- Reads a standard RC PWM channel 5 (1000–2000 µs).
- Arms when PWM exceeds a configurable threshold.
- Disarms when PWM drops below threshold.
- Fully non‑blocking and jitter‑resistant.

### **Servo Expander (PWM Input → Scaled PWM Output)**
- Reads PWM input on **pin 7** from receiver (ELRS / CRSF).
- Supports **50–500 Hz** input frequencies.
- Measures pulse width and input period in real time.
- Output frequency **matches the input frequency**.
- Uses RP2040 **hardware PWM** for rock‑solid timing.
- ISR is ultra‑lightweight and validated for ELRS PWM receivers.

### **Non‑Blocking Architecture**
- MSP parsing  
- PWM arming  
- Servo expansion  
- LED updates
- GPS Updating  
- State machine transitions  

This structure makes the firmware easy to extend, debug, and maintain.

---

## 🧩 Hardware Requirements

- RP2040‑Zero or compatible RP2040 board  
- DJI O3, O4 / Vista / Air Unit / Walksnail Avatar HD UART connection  
- RC PWM receiver (ELRS PWM RX recommended)  

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

5. **Servo Expansion**  
   - Continuously reads PWM input  
   - Measures pulse width + frequency  
   - Outputs scaled PWM on pin 29  
   - Fully independent of arming logic  

---

## 🧪 Tested With

- ELRS PWM receivers (50–500 Hz)  
- DJI O4 Air Unit MSP telemetry
- Walksnail Avatar HD Goggles X
- Servos and gimbal controllers  
- RP2040‑Zero hardware  

---

## 📄 License

MIT License — feel free to modify, extend, and integrate into your own builds.
