🌱 **Autonomous Smart Irrigation Rover with Obstacle Avoidance**

This project is a smart irrigation system powered by an Arduino Uno, combining environmental monitoring, autonomous navigation, and obstacle avoidance. It’s designed to irrigate predefined rows of a field automatically based on soil moisture levels and tank water availability, while intelligently navigating around obstacles using an ultrasonic sensor.


🔧**Features**

Soil Moisture Monitoring using an analog soil sensor.

Water Level Detection from a tank to prevent dry running of the pump.

Autonomous Navigation through 5 predefined rows using motor control.

Obstacle Detection & Avoidance with ultrasonic sensor logic.

Temperature & Humidity Monitoring via DHT11 sensor.

LCD Display shows live sensor readings.

Pump Control based on sensor thresholds.



**SMS Alerts via serial-based GSM logic for:**

Low soil moisture

Low/high water level

High temperature

High humidity

Obstacle Alerts using an onboard LED.

State Machine Logic for modular and extendable robot behavior.



🧠**Core Components**

Arduino Uno

DHT11 Temperature and Humidity Sensor

Soil Moisture Sensor

Water Level Sensor

Ultrasonic Sensor (HC-SR04)

L298N Motor Driver

DC Motors

Relay Module for Water Pump

16x2 I2C LCD Display




**🧩 How It Works**

The robot starts in MOVE_ROW mode and navigates straight along a row for 10 seconds.

If it detects an obstacle (distance < 12 cm), it triggers avoidObstacle() to bypass it.

After each row, it performs a U-turn, switches direction (left or right), and proceeds to the next row.



**While moving, it constantly checks:**


Soil Moisture: If dry, and water is available, the pump is activated.

Water Level: Prevents pump operation if the tank is low.

Temperature/Humidity: Read via DHT11, and updates LCD and serial.

After completing all 5 rows, it enters IDLE mode and stops.

**📟 LCD Display Output**
**Displays real-time:**

Top Line: T:<temp>C H:<hum>%

Bottom Line: S:<soil> W:<water>%

**🔔 Alerts via Serial (for GSM)
Data is formatted and sent to simulate SMS/email alerts like:**

sensornewgsm.php?client=iot2k24145&s1=MOISTURE_LEVEL_LOW&sms=YES&msg=...
You can connect this to a GSM module or IoT server that listens for such formatted messages.

**🛠 Setup Notes**

Make sure to adjust the analog read range mapping for your water level sensor depending on your tank’s real-world min and max readings.

Fine-tune the obstacle avoidance timings if your robot’s geometry is different.

Set soilThreshold and waterThreshold based on calibration.

**🚀 Future Improvements**


Add GPS/GNSS for open field coverage.

Integrate a web dashboard or app for live monitoring.

Use an RTC module for scheduled irrigation.

Switch to a more accurate capacitive soil sensor.
