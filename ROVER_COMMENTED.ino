#include <DHT.h>
#include <LiquidCrystal_I2C.h>

// Sensor and hardware pin setup
#define SOIL_PIN A0  // Aiyo, soil moisture sensor connected to A0, to check if soil is dry or not
#define WATER_PIN A1  // Water level sensor at A1, to see if tank has enough water
#define PUMP_PIN 8  // Water pump control pin, this one turns pump on/off
#define TRIG_PIN 3  // Ultrasonic sensor trigger pin, sends signal to check obstacles
#define ECHO_PIN 4  // Ultrasonic echo pin, gets the reflected signal

// Motor driver pins
#define ENA 5  // Motor A speed control, PWM pin for left motor
#define IN1 6  // Motor A direction 1, for forward/backward
#define IN2 7  // Motor A direction 2, opposite of IN1
#define ENB 11  // Motor B speed control, PWM for right motor
#define IN3 9  // Motor B direction 1, same logic as IN1
#define IN4 10  // Motor B direction 2, for right motor movement

#define DHTPIN 2  // DHT11 sensor pin, for temp and humidity
#define DHTTYPE DHT11  // Using DHT11 sensor, cheap and works fine
DHT dht(DHTPIN, DHTTYPE);  // Creating DHT object, initialized with pin and type

LiquidCrystal_I2C lcd(0x27, 16, 2);  // LCD setup with I2C address 0x27, 16x2 display

// Field configuration
const int totalRows = 5;  // Total rows in the field, robot will move through 5 rows
int currentRow = 0;  // Tracks which row the robot is in
bool movingRight = true;  // Direction flag, true means moving right, false is left

// Threshold values for triggering pump
const int soilThreshold = 500;  // Soil moisture threshold, above 500 means too dry, pump on
const int waterThreshold = 85;  // Water level threshold, below 85% means tank is low

// Movement timing
unsigned long moveStartTime = 0;  // Tracks when robot starts moving in a row
const unsigned long rowMoveDuration = 10000;  // 10 seconds to move along one row
bool rowMovementStarted = false;  // Flag to check if row movement has started

int count=0;  // Counter for moisture alerts, to limit SMS spam
int count1=0;  // Counter for water level alerts
int count2=0;  // Counter for humidity alerts
int count3=0;  // Counter for temperature alerts

enum State { MOVE_ROW, TURN_NEXT_ROW, IDLE };  // States for robot: move in row, turn, or stop
State currentState = MOVE_ROW;  // Start with moving in row

void setup() {
  Serial.begin(9600);  // Start serial for debugging, 9600 baud rate
  dht.begin();  // Initialize DHT11 sensor

  lcd.init();  // Start the LCD
  lcd.backlight();  // Turn on backlight, looks cool
  lcd.clear();  // Clear the screen
  lcd.print("Initializing...");  // Show startup message
  delay(2000);  // Wait 2 seconds for drama
  lcd.clear();  // Clear again for fresh display

  pinMode(PUMP_PIN, OUTPUT);  // Pump pin as output
  pinMode(TRIG_PIN, OUTPUT);  // Ultrasonic trigger as output
  pinMode(ECHO_PIN, INPUT);  // Ultrasonic echo as input
  pinMode(13, OUTPUT);  // Onboard LED for alerts, like warning light

  // Motor pin configuration
  pinMode(ENA, OUTPUT);  // Speed control for motor A
  pinMode(IN1, OUTPUT);  // Direction pins for motor A
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);  // Speed control for motor B
  pinMode(IN3, OUTPUT);  // Direction pins for motor B
  pinMode(IN4, OUTPUT);

  stopMotors();  // Stop motors initially, safety first
  delay(1000);  // Wait 1 second before starting
}

void loop() {
  checkDHT();  // Check temperature and humidity, like weather report
  checkSoilAndWater();  // Check soil moisture and water level, important for irrigation

  switch (currentState) {  // State machine for robot movement
    case MOVE_ROW:
      if (!rowMovementStarted) {  // If row movement hasn’t started
        if (checkObstacle()) {  // Check for obstacles using ultrasonic
          avoidObstacle();  // If obstacle found, avoid it
        } else {
          moveForward();  // No obstacle, move forward
          moveStartTime = millis();  // Start timer for row movement
          rowMovementStarted = true;  // Mark row movement as started
        }
      } else {
        // Check if row movement time is over
        if (millis() - moveStartTime >= rowMoveDuration) {
          stopMotors();  // Stop at end of row
          rowMovementStarted = false;  // Reset flag
          currentState = TURN_NEXT_ROW;  // Move to turning state
        } else if (checkObstacle()) {  // Mid-row obstacle check
          unsigned long obstacleStart = millis();  // Start timer for avoidance
          avoidObstacle();  // Avoid the obstacle
          unsigned long obstacleTime = millis() - obstacleStart;  // Time taken to avoid
          moveStartTime += obstacleTime;  // Adjust row timer
          moveForward();  // Resume moving forward
        }
      }
      break;

    case TURN_NEXT_ROW:
      // Logic for U-turn at row end
      if (currentRow < totalRows - 1) {  // If not last row
        if (movingRight) {  // Moving right, do right U-turn
          turnRight();
          delay(2000);  // Turn for 2 seconds
          stopMotors();
          delay(300);  // Small pause
          moveForward();  // Move to align for next row
          delay(3000);  // Move for 3 seconds
          stopMotors();
          delay(300);
          turnRight();  // Complete U-turn
          delay(2000);
          stopMotors();
          delay(300);
        } else {  // Moving left, do left U-turn
          turnLeft();
          delay(2000);
          stopMotors();
          delay(300);
          moveForward();
          delay(3000);
          stopMotors();
          delay(300);
          turnLeft();
          delay(2000);
          stopMotors();
          delay(300);
        }
        movingRight = !movingRight;  // Switch direction for next row
        currentRow++;  // Move to next row
        currentState = MOVE_ROW;  // Back to moving state
      } else {
        currentState = IDLE;  // All rows done, stop robot
      }
      break;

    case IDLE:
      stopMotors();  // Just chill, no movement
      break;
  }
}

// Reads moisture and water tank level and controls pump
void checkSoilAndWater() {
  int soilValue = analogRead(SOIL_PIN);  // Read soil moisture value
  int waterValue = analogRead(WATER_PIN);  // Read water level value

  // Convert water level to percentage
  int percentage = map(constrain(waterValue, 500, 700), 500, 700, 0, 100);  // Map water level to 0-100%

  // Send data to serial for server
  Serial.print("sensornewgsm.php?client=iot2k24145&s1=");
  Serial.print(soilValue);  // Send soil value
  Serial.print("&s2=NA&s3=NA&s4=NA&s5=NA&sms=NA&msg=NA#");
  delay(500);  // Small delay for serial
  Serial.print("sensornewgsm.php?client=iot2k24145&s2=");
  Serial.print(waterValue);  // Send water level
  Serial.print("&s1=NA&s3=NA&s4=NA&s5=NA&sms=NA&msg=NA#");
  delay(500);
  Serial.print(" (");
  Serial.print(percentage);  // Print percentage for debugging
  Serial.println("%)");

  // Control pump based on conditions
  if (soilValue > soilThreshold && count<=2) {
    digitalWrite(PUMP_PIN, LOW);  // Turn off pump if soil too dry
  }
  else if(percentage < waterThreshold){
    digitalWrite(PUMP_PIN, LOW);  // Turn off pump if water level low
  }
  else{
    digitalWrite(PUMP_PIN, HIGH);  // Turn on pump otherwise
  }

  // Send alerts for low moisture
  if (soilValue > soilThreshold && count<=2) {
    Serial.print("sensornewgsm.php?client=iot2k24145&s1=");
    Serial.print("MOISTURE_LEVEL_LOW");  // Send low moisture alert
    Serial.print("&s2=NA&s3=NA&s4=NA&s5=NA&sms=YES&msg=MOISTURE_LEVEL_LOW#");
    count+=1;  // Increment alert counter
    delay(500);
  }
  if (percentage >= waterThreshold && count<=2) {
    Serial.print("sensornewgsm.php?client=iot2k24145&s2=");
    Serial.print("WATER_LEVEL_HIGH");  // Send  water alert
    Serial.print("&s1=NA&s3=NA&s4=NA&s5=NA&sms=YES&msg=WATER_LEVEL_HIGH#");
    count1 += 1;  // Increment water alert counter
    delay(500);
  }

  // Turn on LED if tank is full
  digitalWrite(13, (percentage >= 100) ? HIGH : LOW);  // LED on if tank full

  // Update LCD with soil and water data
  lcd.setCursor(0, 1);
  lcd.print("S:");
  lcd.print(soilValue);  // Show soil moisture
  lcd.print(" W:");
  lcd.print(percentage);  // Show water percentage
  lcd.print("% ");
}

// Monitors temperature and humidity
void checkDHT() {
  float temp = dht.readTemperature();  // Read temperature
  float hum = dht.readHumidity();  // Read humidity

  if (isnan(temp) || isnan(hum)) {  // Check if sensor reading failed
    Serial.println("DHT11 Error reading values");  // Print error
    return;
  }

  // Print temp and humidity to serial
  Serial.print("Temp: ");
  Serial.print(temp);
  Serial.print("°C ");
  Serial.print("Humidity: ");
  Serial.print(hum);
  Serial.println("%");

  // Send temp and humidity to server
  Serial.print("sensornewgsm.php?client=iot2k24145&s4=");
  Serial.print(temp);
  Serial.print("&s1=NA&s2=NA&s3=NA&s5=NA&sms=NA&msg=NA#");
  delay(500);
  Serial.print("sensornewgsm.php?client=iot2k24145&s3=");
  Serial.print(hum);
  Serial.print("&s1=NA&s2=NA&s4=NA&s5=NA&sms=NA&msg=NA#");
  delay(500);

  // Alert for high humidity
  if (hum > 90 && count<=2) {
    Serial.print("sensornewgsm.php?client=iot2k24145&s3=");
    Serial.print("HUMIDITY_LEVEL_HIGH");  // Send high humidity alert
    Serial.print("&s1=NA&s2=NA&s4=NA&s5=NA&sms=YES&msg=HUMIDITY_LEVEL_HIGH#");
    count2 += 1;  // Increment humidity alert counter
    delay(500);
  }

  // Alert for high temperature
  if (temp > 38 && count<=2) {
    Serial.print("sensornewgsm.php?client=iot2k24145&s4=");
    Serial.print("HIGH_TEMPERATURE");  // Send high temperature alert
    Serial.print("&s1=NA&s2=NA&s3=NA&s5=NA&sms=YES&msg=HIGH_TEMPERATURE#");
    count3 += 1;  // Increment temp alert counter
    delay(500);
  }

  // Update LCD with temp and humidity
  lcd.setCursor(0, 0);
  lcd.print("T:");
  lcd.print((int)temp);  // Show temperature value
  lcd.print("C H:");
  lcd.print((int)hum);  // Show humidity value
  lcd.print("%  ");
}

// Motor movement helpers
void moveForward() {
  digitalWrite(IN1, HIGH);  // Set motor A forward
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);  // Set motor B forward
  digitalWrite(IN4, LOW);
  analogWrite(ENA, 150);  // Set speed for motor A
  analogWrite(ENB, 150);  // Set speed for motor B
}

void moveBackward() {
  digitalWrite(IN1, LOW);  // Set motor A backward
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);  // Set motor B backward
  digitalWrite(IN4, HIGH);
  analogWrite(ENA, 150);  // Same speed
  analogWrite(ENB, 150);
}

void turnRight() {
  digitalWrite(IN1, HIGH);  // Motor A forward
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);  // Motor B backward
  digitalWrite(IN4, HIGH);
  analogWrite(ENA, 150);  // Speed for turning
  analogWrite(ENB, 150);
}

void turnLeft() {
  digitalWrite(IN1, LOW);  // Motor A backward
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);  // Motor B forward
  digitalWrite(IN4, LOW);
  analogWrite(ENA, 150);  // Speed for turning
  analogWrite(ENB, 150);
}

void stopMotors() {
  digitalWrite(IN1, LOW);  // Stop both motors
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

// Ultrasonic obstacle detection
bool checkObstacle() {
  digitalWrite(TRIG_PIN, LOW);  // Clear trigger pin
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);  // Send ultrasonic pulse
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH, 30000);  // Measure echo time
  if (duration == 0) {
    Serial.println("Ultrasonic timeout");  // If no echo, print error
    return false;
  }

  int distance = duration * 0.034 / 2;  // Calculate distance in cm
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");

  if (distance < 12) {
    Serial.println("WARNING: Obstacle very close!");  // Alert if too close
    digitalWrite(13, HIGH);  // Turn on LED
  } else {
    digitalWrite(13, LOW);  // LED off if no obstacle
  }

  return distance <= 12;  // Return true if obstacle within 12 cm
}

// Simple path to avoid obstacles
void avoidObstacle() {
  stopMotors();  // Stop first
  delay(300);  // Small pause
  moveBackward();  // Back up a bit
  delay(1000);
  turnLeft();  // Turn left to avoid
  delay(2000);
  moveForward();  // Move forward
  delay(1000);
  turnRight();  // Turn right
  delay(2000);
  moveForward();  // Move again
  delay(2000);
  turnRight();  // Another right turn
  delay(2000);
  moveForward();  // Move forward
  delay(1000);
  turnLeft();  // Back to original direction
  delay(2000);
  stopMotors();  // Stop after avoiding
  delay(300);
}