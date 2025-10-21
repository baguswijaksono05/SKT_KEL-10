# 🌐 Edge Gateway and Integration of DWSIM, Rust Backend, and Cloud ThingsBoard

## 📘 Overview
This project presents the design and implementation of an **Edge Gateway System** that integrates:

- **DWSIM** (chemical process simulator)  
- **Rust Backend** (edge and cloud communication handler)  
- **ThingsBoard Cloud Platform** (for visualization and monitoring)  

The integration enables real-time synchronization of process data between simulation, embedded systems, and cloud infrastructure.  
This architecture supports **Digital Twin** and **Industrial IoT (IIoT)** applications by ensuring secure, efficient, and scalable data communication.

---

## 🧩 System Architecture
The proposed architecture follows a **Distributed Control System (DCS)** structure, where data is collected from physical sensors and simulation environments, processed at the edge, and visualized in the cloud.

### Components
| Component | Description |
|------------|-------------|
| **ESP32-S3** | Acts as the main edge gateway; handles Modbus RS-485 communication, data processing, and MQTT transmission. |
| **SHT20 Sensor** | Measures temperature and humidity via I²C. Compact, low-power, and reliable. |
| **RS-485 Module (MAX485)** | Enables stable serial communication between sensors and the microcontroller using Modbus RTU protocol. |
| **Rust Backend** | Manages secure data processing, MQTT communication, and InfluxDB storage. Optimized for concurrency and memory safety. |
| **InfluxDB** | Time-series database used for logging and analyzing sensor and simulation data. |
| **ThingsBoard** | Cloud IoT platform used for visualization, monitoring, and dashboard management. |
| **DWSIM** | Chemical process simulator that generates virtual process data (temperature, flow, pressure, etc.) to integrate with the real-time system. |
| **Relay & Buzzer** | Actuators for process control and alert indication. |

---

## ⚙️ Methodology

### 1. Component Design
The **ESP32-S3** is programmed as an **Edge Gateway** using **Embedded Rust**.  
It connects to sensors (via RS-485 Modbus) and sends processed data to:

- **InfluxDB** → for real-time storage  
- **ThingsBoard** → for visualization through MQTT  
- **DWSIM Integration** → for simulation data synchronization  

### 2. System Design
Data flow involves:
1. Sensor and simulated data acquisition (ESP32-S3, DWSIM, Python scripts)  
2. Processing and filtering by the Rust backend  
3. Real-time visualization in the ThingsBoard dashboard  
4. Historical data stored in InfluxDB for trend analysis and validation  

This ensures **two-way communication** between the physical system and its digital representation.

---

## 💻 Software & Tools Used
| Tool | Function |
|------|-----------|
| **Rust (esp-idf-svc)** | Embedded programming for ESP32-S3. |
| **Python** | Gateway integration and XML data parsing from DWSIM. |
| **InfluxDB** | Time-series database for cloud and local storage. |
| **ThingsBoard** | Real-time dashboard for IoT visualization. |
| **DWSIM** | Process simulation software for virtual plant modeling. |
| **VSCode** | Development environment for Python and Rust. |

---

## 🔗 Communication Protocols
- **MQTT** → For cloud data publishing (ThingsBoard integration)  
- **Modbus RTU / TCP** → For communication between ESP32-S3 and field devices  
- **HTTP API** → For Rust–InfluxDB data transfer  

---

## 📊 Results

### System Performance
- Average latency: **~120 ms** between DWSIM updates and ThingsBoard visualization  
- Stable performance during continuous operation (no data loss)  
- Efficient CPU and memory usage due to Rust optimization  

### Cloud Dashboard
- ThingsBoard displayed **temperature** and **humidity** trends in real time  
- Interactive charts with **alert rules** for threshold violations  
- Bi-directional communication supports potential control feedback  

---

## 🧠 Key Findings
- The **Rust backend** provided superior performance and memory safety compared to traditional Python gateways.  
- The **Edge Gateway** effectively synchronized simulated and real sensor data.  
- Integration proved feasible for **Digital Twin** and **Industrial IoT** applications.  

---

## ✅ Conclusion
The project successfully implemented a **fully integrated IoT system** combining simulation, edge processing, and cloud visualization.  

### Key Achievements:
- Real-time data synchronization between **DWSIM**, **ESP32-S3**, and **ThingsBoard**  
- Reliable communication via **MQTT** and **Modbus/TCP** protocols  
- Stable and efficient data handling using the **Rust-based backend**  

This architecture lays the groundwork for **smart industrial systems** that combine accuracy, computational efficiency, and scalability.

---

## 🚀 Future Work
- Integration of **AI-based predictive analytics** within Rust backend for autonomous decision-making  
- Implementation of **closed-loop control systems** for automatic process response  
- Testing with additional sensors and higher communication loads in industrial scenarios  

---

## 👥 Group 10
| Name | NRP |
|------|-----|
| Bagus Wijaksono | 2042231039 |
| Ziyad Zakiy Permana | 2042231077 |
