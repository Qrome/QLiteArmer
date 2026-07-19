# QLiteArmer — HD Armer and Full OSD with Dual‑Core RP2040 Telemetry MSP DisplayPort Processor

![QLiteArmer Custom PCB — Fully Assembled](images/QLiteArmer_Details.png)

QLiteArmer is a dual‑core RP2040‑based telemetry OSD and HD arming system designed for FPV pilots, RC FPV vehicles, RC builders, and embedded developers.  
It provides:

- Full Betaflight‑style OSD for DJI O3/O4 and Walksnail Avatar HD  
- GPS + barometer telemetry processing  
- CRSF/ELRS channel breakout  
- PWM‑based arming (Channel 5)  
- 8‑channel servo expansion  
- Deterministic, non‑blocking architecture  

QLiteArmer is an OSD and arming module — arming is **controlled by RC PWM Channel 5**.

**[Get the QLiteArmer PCB Short Kit Here](https://www.etsy.com/shop/BSRC?section_id=59459029)**

---

![QLiteArmer Custom PCB — Fully Assembled](images/OSD_Sample_01.jpg)

## ✨ Features

### **Full OSD Engine (MSP DisplayPort)**
QLiteArmer includes a complete Betaflight‑compatible OSD renderer supporting:

- Satellite count  
- Pack voltage  
- Cell voltage  
- Vertical speed  
- RC link quality (ELRS LQ)  
- Distance from home  
- Altitude  
- Ground speed  
- Total trip distance  
- Latitude / longitude  
- Home arrow (16‑direction)  
- Ground radar of home  
- Crosshair  
- Flight timer  
- Throttle value  
- Compass heading

Compatible with:

- **DJI O3 / O4 Air Unit**
- **Walksnail Avatar HD (Goggles X / VTX)**

Uses Betaflight character mappings and custom glyphs for home arrow rendering.

---

## 🖼 Betaflight-Compatible Character Map (Walksnail Avatar HD)

This project includes a full custom character map matching Betaflight’s DisplayPort glyph layout, adapted for Walksnail Avatar HD.

<img src="images/first_column_ascii_map.png" alt="Betaflight-Compatible Character Map (Walksnail Avatar HD)" style="background-color: black;">
---

## 🎮 PWM‑Based Arming (Channel 5)

Arming is now **100% PWM‑based**:

- Uses RC Channel 5 (default)
- Arms when PWM > threshold (default: 1700 µs)
- Disarms when PWM < threshold  

This ensures predictable, pilot‑controlled arming behavior.

---

## 🛰 Telemetry System

### **GPS Telemetry**
- Autodetects baud rate  (9600, 38400, 57600, 115200)
- Home position lock  
- Distance from home  
- Bearing to home  
- Ground radar showing home  
- Ground speed  
- Latitude / longitude  
- Total trip distance  
- GPS fix + satellite count  

### **Barometer (BMP280)**
- Altitude (cm)  
- Vertical speed (cm/s)  

### **Battery Monitoring**
- VBAT via ADC  
- Configurable voltage divider  
- Pack voltage displayed in OSD  
- Per cell voltage  

### **Units**
- Metric or Imperial (configurable)

---

## 🧩 Servo Expansion (8‑Channel PWM Output)

QLiteArmer includes a hardware‑accurate servo expander:

- 8 PWM outputs  
- Per‑channel min/max mapping  
- Per‑channel failsafe values  
- Frequency‑matched output  
- CRSF/ELRS channel passthrough  
- Ultra‑stable RP2040 hardware PWM  

Define individual channel range and failsafe in the config.h file:
```C
static const ChannelMap CH_MAP[8] = {
    {988, 2012, 1500},   // CH1
    {988, 2012, 1500},   // CH2
    {988, 2012, 1000},   // CH3 (Throttle failsafe = 1000)
    {988, 2012, 1500},   // CH4 
    {988, 2012, 1500},   // CH5
    {500, 2500, 1500},   // CH6 (expanded range)
    {988, 2012, 1500},   // CH7
    {988, 2012, 1500}    // CH8
};
```

---

## ⚙ Pin Map

| Function | Pin(s) | Notes |
| --- | --- | --- |
| **CRSF / ELRS UART** | GP8 (TX), GP9 (RX) | Primary RC link |
| **MSP UART (VTX)** | GP0 (TX), GP1 (RX) | DJI / Walksnail |
| **I²C (BMP280)** | GP4 (SDA), GP5 (SCL) | Barometer |
| **VBAT ADC** | GP26 | Voltage divider |
| **PWM CH1–CH8** | 13, 12, 11, 10, 7, 6, 3, 2 | Servo/motor outputs |
| **RGB LED** | GP16 | Onboard WS2812 |
| **GPS** | GP14 (TX), GP15 (RX) | 4‑wire GPS |

---
## Software & Environment Setup

This project is built for the Raspberry Pi Pico / RP2040 / RP2350 architecture using the Arduino IDE. Follow the steps below to configure your development environment.

### 1. Install the Board Support Package (BSP)
You must use the Arduino Pico core maintained by Earle F. Philhower, III. 

1. Open the Arduino IDE.
2. Navigate to **File** -> **Preferences**.
3. Locate the **Additional Boards Manager URLs** field and paste the following URL:
   ```
   https://github.com/earlephilhower/arduino-pico/releases/download/global/package_rp2040_index.json
   ```
4. Click **OK**.
5. Go to **Tools** -> **Board** -> **Boards Manager...**
6. Search for `Pico` or `Philhower` and install **Raspberry Pi Pico/RP2040/RP2350** by *Earle F. Philhower, III*.
7. Once installed, go to **Tools** -> **Board** -> **Raspberry Pi Pico/RP2040** and select **Waveshare RP2040 Zero**.

### 2. Install Required Libraries
Open the Arduino Library Manager (**Tools** -> **Manage Libraries...** or press `Ctrl+Shift+I` / `Cmd+Shift+I`) to search for and install:

* **Adafruit BMP280 Library** (by Adafruit)
* **NeoPixelBus** (by Makuna)

> 💡 **Note:** When installing the `Adafruit BMP280` library, the IDE will ask to install required dependencies (like `Adafruit BusIO` and `Adafruit Unified Sensor`). Make sure to select **Install All** so everything compiles successfully.
---

## 🛠 Hardware Overview

### **QLiteArmer Custom PCB — Bare Board Available Soon**
![QLiteArmer Custom PCB — Bare Board](images/QLiteArmer_board_00.png)

### **QLiteArmer Custom PCB — Fully Assembled**
![QLiteArmer Custom PCB — Fully Assembled](images/QLiteArmer_board_01.png)

## 🧰 Parts List (with Affiliate Links)

These are the recommended components for building a complete QLiteArmer module.  
All links are affiliate links that help support the project at no additional cost.

### **Core Components**
- **QLiteAmer PCB Board by Qrome**  
  https://www.etsy.com/shop/BSRC?section_id=59459029

- **RP2040‑Zero**  
  https://amzn.to/4v0oCKq

- **M100-5883 GPS (115200 bps 5Hz - 10Hz)**  
  https://amzn.to/4weRRKt

- **HGLRC ExpressLRS 915 MHz Receiver**  
  https://amzn.to/4w56T5E

- **BMP280‑3.3 Atmospheric Pressure Sensor**  
  https://amzn.to/44y8Onq

### **Power & Regulation**
- **3A Mini DC‑DC Buck Converter (5.5–30 V → 5 V)**  
  https://amzn.to/4wcNi3h

### **Passive Components**
- **30 kΩ Resistor (¼ W)**  
  https://amzn.to/3StSnWB

- **7.5 kΩ Resistor (¼ W)**  
  https://amzn.to/440p8Nx

- **0.1 µF (100 nF) Ceramic Capacitors (104)**  
  https://amzn.to/4xO8TAQ

- **1000 µF 25 V Electrolytic Capacitor (optional)**  
  https://amzn.to/4vtyai8

### **Headers & Connectors**
- **Pin Header, 2.54mm 40Pin Male and Female Header Pins**  
  https://amzn.to/4oQfh6B

### **Other Recommended Items**
- **RADIOMASTER TX15 Max**  
  https://amzn.to/4vCAtQa

- **Walksnail Avatar HD FPV Goggles X**  
  https://amzn.to/4f2cXWN

- **Walksnail Avatar GT2 Kit – Air Unit**  
  https://amzn.to/4geaycN  


[![QLiteArmer OSD and HD Armer Detailed Build - RP2040 Zero FPV System](https://img.youtube.com/vi/8tNotUI8HCQ/0.jpg)](https://www.youtube.com/watch?v=8tNotUI8HCQ)

---

## 🚀 How It Works (High‑Level)

1. **Boot**
   - System initializes dual‑core architecture  
   - Blue LED indicates boot state  

2. **Telemetry Acquisition**
   - Core 1 handles GPS, barometer, battery, and OSD  
   - Core 0 handles CRSF/ELRS and PWM output  

3. **OSD Rendering**
   - MSP DisplayPort frames generated in real time  
   - Betaflight‑compatible glyphs  
   - Home arrow computed from GPS bearing  

4. **Arming**
   - PWM Channel 5 controls arming  
   - Red LED is holding and ready to arm  
   - Green LED is armed with high power and recording  
   - Amber LED no VTX detected

5. **Servo Expansion**
   - CRSF channels mapped to PWM outputs  
   - Failsafe values applied on link loss  

---

## 🧠 System Architecture (Technical)

### **Core 0 (Time‑Critical)**
- CRSF/ELRS UART  
- PWM output generation  
- MSP heartbeat  
- Channel mapping  
- Failsafe handling  

### **Core 1 (Telemetry + OSD)**
- MSP DisplayPort  
- GPS parsing  
- Barometer updates  
- Battery ADC  
- OSD composition  
- State machine  
- LED driver  

### **SerialPIO Notes**
- SerialPIO is used for GPS  
- Requires careful initialization order  
- LED initialization must occur before SerialPIO  

### **RP2040 vs RP2350**
- Both fully supported  
- LED color order differs between boards (Red and Green are swapted)  
- Auto‑detected via compile‑time selection  

---

## 🔧 Configuration Reference

### **PWM Channel Mapping**
Each channel has:
- `minUs`
- `maxUs`
- `failsafeUs`

Example (from config.h):
```C
static const ChannelMap CH_MAP[8] = {
    {988, 2012, 1500},   // CH1
    {988, 2012, 1500},   // CH2
    {988, 2012, 1000},   // CH3 (Throttle failsafe = 1000)
    {988, 2012, 1500},   // CH4 
    {988, 2012, 1500},   // CH5
    {500, 2500, 1500},   // CH6 (expanded range)
    {988, 2012, 1500},   // CH7
    {988, 2012, 1500}    // CH8
};
```

### **Arming Settings**
- `PWM_ARM_CHANNEL = 4` (Channel 5 -- ZERO based mapping)  
- `PWM_ARM_THRESHOLD = 1700`  
- `PWM_NO_SIGNAL_US = 900`  

### **Battery Divider**
Default:
- R1 = 30k  
- R2 = 7.5k  

### **Telemetry Rates**
- Battery: 5 Hz  
- Altitude: 5 Hz  

### **Units**
- Metric or Imperial  

### **Timing**
- VTX detection timeout: 5 minutes  
- Heartbeat: 200 ms  

### **LED**
- LED pin: GP16  
- LED count: 1  
- RP2040/RP2350 color order handled internally  

### **Home Arrow Glyph Table**
Rows 96–111  
16 directions  
22.5° increments  
CCW orientation  

---

## 🧪 Tested Hardware

- RP2040‑Zero  
- RP2350-Zero  
- DJI O3 / O4 Air Unit  
- Walksnail Avatar HD  
- ELRS receivers  
- BMP280 barometer  
- Standard GPS modules  

---

## 📄 License

MIT License — free to modify, extend, and integrate.

