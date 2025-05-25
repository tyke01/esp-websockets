# Arduino Temperature & Vibration Monitor

A real-time temperature and vibration monitoring system using ESP32, LM35 temperature sensor, and MPU6050 accelerometer/gyroscope. The system provides live data streaming via WebSocket to a connected web interface with visual alerts and LED indicators.

## 🌟 Features

- **Real-time Temperature Monitoring**: LM35 analog temperature sensor with calibration
- **Vibration Detection**: MPU6050 6-axis motion sensor for acceleration and gyroscope data
- **Live Data Streaming**: WebSocket server for real-time data transmission
- **Visual Alerts**: 4-level LED warning system based on temperature and vibration thresholds
- **Web Interface**: Real-time dashboard for monitoring sensor data
- **WiFi Connectivity**: Wireless data transmission and remote monitoring
- **JSON Data Format**: Structured data output for easy integration

## 📋 Hardware Requirements

### Main Components
- **ESP32 Development Board** (or compatible)
- **LM35 Temperature Sensor**
- **MPU6050 Accelerometer/Gyroscope Module**
- **4x LEDs** (for status indicators)
- **4x Resistors** (220Ω - 330Ω for LEDs)
- **Breadboard and Jumper Wires**

### Pin Connections

| Component | ESP32 Pin | Notes |
|-----------|-----------|-------|
| LM35 VCC | 3.3V | Power supply |
| LM35 GND | GND | Ground |
| LM35 OUT | GPIO 32 | Analog input |
| MPU6050 VCC | 3.3V | Power supply |
| MPU6050 GND | GND | Ground |
| MPU6050 SDA | GPIO 21 | I2C Data |
| MPU6050 SCL | GPIO 22 | I2C Clock |
| LED 1 | GPIO 16 | ≥35°C or ≥12 m/s² |
| LED 2 | GPIO 2 | ≥40°C or ≥16 m/s² |
| LED 3 | GPIO 15 | ≥45°C or ≥20 m/s² |
| LED 4 | GPIO 4 | ≥50°C or ≥24 m/s² |

## 🔧 Software Requirements

### Arduino IDE Setup
1. Install [Arduino IDE](https://www.arduino.cc/en/software)
2. Add ESP32 board support:
   - Go to File → Preferences
   - Add `https://dl.espressif.com/dl/package_esp32_index.json` to Additional Board Manager URLs
   - Go to Tools → Board → Board Manager
   - Search and install "esp32"

### Required Libraries
Install these libraries via Library Manager (Tools → Manage Libraries):

```
- Adafruit MPU6050
- Adafruit Unified Sensor
- WebSocketsServer by Markus Sattler
- ArduinoJson by Benoit Blanchon
```

## 🚀 Installation & Setup

### 1. Hardware Assembly
1. Connect components according to the pin connection table
2. Ensure proper power supply connections (3.3V and GND)
3. Add appropriate resistors in series with LEDs

### 2. Software Configuration
1. Clone this repository:
   ```bash
   git clone https://github.com/yourusername/arduino-temp-vibration-monitor.git
   ```

2. Open `lm35-temp-test.ino` in Arduino IDE

3. Update WiFi credentials:
   ```cpp
   const char* ssid = "Your_WiFi_Network";
   const char* password = "Your_WiFi_Password";
   ```

4. Adjust calibration if needed:
   ```cpp
   #define TEMP_OFFSET -25.0  // Adjust based on your sensor
   ```

### 3. Upload and Run
1. Select your ESP32 board: Tools → Board → ESP32 Dev Module
2. Select the correct port: Tools → Port
3. Upload the code to your ESP32
4. Open Serial Monitor (115200 baud) to view connection status

## 📊 Data Output

The system outputs JSON data via WebSocket with the following structure:

```json
{
  "timestamp": 12345,
  "lm35_temp_c": 25.67,
  "lm35_temp_f": 78.21,
  "mpu_temp_c": 26.45,
  "vibration": 10.23,
  "acceleration": {
    "x": 0.12,
    "y": -0.45,
    "z": 9.81
  },
  "rotation": {
    "x": 0.02,
    "y": -0.01,
    "z": 0.03
  },
  "leds": {
    "led1": false,
    "led2": false,
    "led3": false,
    "led4": false
  },
  "alerts": {
    "temp_alert": false,
    "vibration_alert": false
  },
  "connected_clients": 1,
  "wifi_connected": true
}
```

## 🌐 Web Interface

The system includes a WebSocket server that broadcasts sensor data to connected clients. You can connect to the WebSocket at:

```
ws://[ESP32_IP_ADDRESS]:81
```

### Connecting to the Dashboard
1. Find your ESP32's IP address in the Serial Monitor after successful WiFi connection
2. Open your web browser and navigate to your web interface
3. The interface will automatically connect to `ws://[ESP32_IP]:81`
4. Real-time data will be displayed with visual indicators

## ⚙️ Configuration

### Temperature Thresholds
```cpp
#define TEMP_THRESHOLD_1 35.0  // °C
#define TEMP_THRESHOLD_2 40.0  // °C
#define TEMP_THRESHOLD_3 45.0  // °C
#define TEMP_THRESHOLD_4 50.0  // °C
```

### Vibration Thresholds
```cpp
#define VIBRATION_THRESHOLD_1 12.0  // m/s²
#define VIBRATION_THRESHOLD_2 16.0  // m/s²
#define VIBRATION_THRESHOLD_3 20.0  // m/s²
#define VIBRATION_THRESHOLD_4 24.0  // m/s²
```

### Sensor Reading Interval
```cpp
const unsigned long sensorInterval = 100; // milliseconds
```

## 🔍 Monitoring & Debugging

### Serial Monitor Output
The system provides detailed logging via Serial Monitor:
- WiFi connection status
- WebSocket client connections/disconnections
- Real-time sensor readings
- Error messages and debugging information

### LED Status Indicators
- **LED 1**: Temperature ≥35°C OR Vibration ≥12 m/s²
- **LED 2**: Temperature ≥40°C OR Vibration ≥16 m/s²
- **LED 3**: Temperature ≥45°C OR Vibration ≥20 m/s²
- **LED 4**: Temperature ≥50°C OR Vibration ≥24 m/s²

## 🛠️ Troubleshooting

### Common Issues

**WiFi Connection Failed**
- Check SSID and password
- Ensure ESP32 is within WiFi range
- Verify network supports 2.4GHz (ESP32 doesn't support 5GHz)

**Sensor Not Detected**
- Check wiring connections
- Verify power supply (3.3V)
- Ensure I2C connections for MPU6050 (SDA/SCL)

**WebSocket Connection Issues**
- Verify ESP32 IP address
- Check firewall settings
- Ensure port 81 is not blocked

**Inaccurate Temperature Readings**
- Adjust `TEMP_OFFSET` calibration value
- Check LM35 wiring and power supply
- Allow sensor to stabilize after power-on

## 📈 Future Enhancements

- [ ] Data logging to SD card
- [ ] Email/SMS alerts for critical thresholds
- [ ] Historical data visualization
- [ ] Mobile app integration
- [ ] Multiple sensor support
- [ ] Cloud data storage integration

## 🤝 Contributing

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/AmazingFeature`)
3. Commit your changes (`git commit -m 'Add some AmazingFeature'`)
4. Push to the branch (`git push origin feature/AmazingFeature`)
5. Open a Pull Request

## 📄 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## 👨‍💻 Author

**Your Name**
- GitHub: [@yourusername](https://github.com/yourusername)
- Email: your.email@example.com

## 🙏 Acknowledgments

- Adafruit for excellent sensor libraries
- ESP32 community for comprehensive documentation
- Arduino IDE team for the development environment

---

**⭐ If this project helped you, please give it a star!**
