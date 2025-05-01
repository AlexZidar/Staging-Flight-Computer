#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP3XX.h>
#include <SD.h>
#include <PWMServo.h>  // Teensy-specific servo library

// Pin definitions
const int SERVO_PIN = 33;              // Servo connected to pin 33
const int SD_CS_PIN = BUILTIN_SDCARD;  // Teensy 4.1 has built-in SD card

// Configuration parameters
const float LAUNCH_THRESHOLD = 4.0;     // g-force threshold for launch detection (adjust as needed)
const float DEPLOYMENT_ALTITUDE = 381.0; // meters (adjust as needed)
const int BMP_WARMUP_READINGS = 5;      // Number of initial readings to discard
const int LOG_INTERVAL_MS = 20;         // Log data every 20ms (50Hz)
const int SERIAL_PRINT_INTERVAL = 1000; // Print debug info every 1000ms

// Servo movement parameters
const int SERVO_START_POS = 0;         // Initial servo position (degrees)
const int SERVO_DEPLOYED_POS = 180;    // Fully deployed position (degrees)
const int SERVO_STEP_DELAY_MS = .5;     // Delay between each degree of movement (ms)
const int SERVO_STEP_SIZE = 3;         // Increment size for servo movement (degrees)

// Global variables
Adafruit_MPU6050 mpu;
Adafruit_BMP3XX bmp;
PWMServo deploymentServo;  // Using Teensy PWMServo
File dataFile;

float baselineAltitude = 0;
bool launchDetected = false;
bool servoDeployed = false;
bool servoDeploymentInProgress = false;
int currentServoPos = SERVO_START_POS;
unsigned long lastLogTime = 0;
unsigned long lastSerialPrintTime = 0;
unsigned long lastServoMoveTime = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000);  // Wait for serial console but continue after 3 seconds
  
  // Initialize I2C on Wire1 (pins 18/19 on Teensy 4.1)
  Wire1.begin();
  
  // Initialize MPU6050
  Serial.println("Initializing MPU6050...");
  if (!mpu.begin(0x68, &Wire1)) {  // Default address 0x68, using Wire1
    Serial.println("MPU6050 initialization failed!");
    while (1) yield();
  }
  
  // Configure MPU6050
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  
  // Initialize BMP390
  Serial.println("Initializing BMP390...");
  if (!bmp.begin_I2C(0x77, &Wire1)) {  // Default address 0x77, using Wire1
    Serial.println("BMP390 initialization failed!");
    while (1) yield();
  }
  
  // Configure BMP390 for maximum polling rate with moderate filtering
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_4X); // Moderate oversampling for balanced readings
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);    // Moderate oversampling for balanced readings
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);       // Medium filter coefficient for smoother readings
  bmp.setOutputDataRate(BMP3_ODR_200_HZ);               // Maximum output data rate (200Hz)
  
  // Initialize SD card
  Serial.println("Initializing SD card...");
  if (!SD.begin(SD_CS_PIN)) {
    Serial.println("SD card initialization failed!");
    while (1) yield();
  }
  
  // Initialize servo
  deploymentServo.attach(SERVO_PIN);
  deploymentServo.write(SERVO_START_POS);  // Initial position
  currentServoPos = SERVO_START_POS;
  
  // Create new log file with incremental name
  char filename[16];
  int fileNumber = 0;
  
  do {
    sprintf(filename, "FLIGHT_%03d.CSV", fileNumber++);
  } while (SD.exists(filename));
  
  dataFile = SD.open(filename, FILE_WRITE);
  if (dataFile) {
    dataFile.println("Time(ms),AccelMag(g),Pressure(Pa),Altitude(m),LaunchDetected,ServoDeployed,ServoPosition");
    dataFile.flush();
    Serial.print("Logging to: ");
    Serial.println(filename);
  } else {
    Serial.println("Error opening log file!");
    while (1) yield();
  }
  
  // Calibrate BMP390 (establish baseline)
  Serial.println("Calibrating BMP390...");
  calibrateBMP();
  
  Serial.println("System ready!");
}

void loop() {
  unsigned long currentTime = millis();
  
  // Read acceleration data from MPU6050
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  
  // Calculate magnitude of acceleration vector (in g)
  float accelMagnitude = sqrt(a.acceleration.x*a.acceleration.x + 
                              a.acceleration.y*a.acceleration.y + 
                              a.acceleration.z*a.acceleration.z) / 9.8; // Convert m/s² to g
  
  // Read and calculate altitude (optimized for speed)
  float pressure = 0;
  float altitude = 0;
  
  if (bmp.performReading()) {
    pressure = bmp.pressure;
    altitude = bmp.readAltitude(1013.25) - baselineAltitude; // Using the library's altitude calculation
  } else {
    // Only print error message occasionally to avoid cluttering serial output
    if (currentTime - lastSerialPrintTime >= SERIAL_PRINT_INTERVAL) {
      Serial.println("BMP390 reading failed!");
    }
  }
  
  // Launch detection
  if (!launchDetected && accelMagnitude > LAUNCH_THRESHOLD) {
    launchDetected = true;
    Serial.println("Launch detected!");
    Serial.print("Acceleration: ");
    Serial.print(accelMagnitude);
    Serial.print("g, Altitude: ");
    Serial.print(altitude);
    Serial.println("m");
    
    // Log the launch event immediately
    logData(currentTime, accelMagnitude, pressure, altitude);
  }
  
  // Servo deployment logic - based only on altitude threshold
  if (!servoDeployed && !servoDeploymentInProgress && altitude >= DEPLOYMENT_ALTITUDE) {
    // Start servo deployment
    servoDeploymentInProgress = true;
    Serial.print("Deployment triggered at altitude: ");
    Serial.print(altitude);
    Serial.println("m");
    
    // Log the deployment trigger event immediately
    logData(currentTime, accelMagnitude, pressure, altitude);
  }
  
  // Handle incremental servo movement if deployment is in progress
  if (servoDeploymentInProgress && currentTime - lastServoMoveTime >= SERVO_STEP_DELAY_MS) {
    if (currentServoPos < SERVO_DEPLOYED_POS) {
      // Move servo one step towards deployed position
      currentServoPos += SERVO_STEP_SIZE;
      if (currentServoPos > SERVO_DEPLOYED_POS) {
        currentServoPos = SERVO_DEPLOYED_POS; // Prevent overshooting
      }
      deploymentServo.write(currentServoPos);
      lastServoMoveTime = currentTime;
    } else {
      // Servo has reached final position
      servoDeploymentInProgress = false;
      servoDeployed = true;
      Serial.println("Servo fully deployed!");
    }
  }
  
  // Log data at specified interval
  if (currentTime - lastLogTime >= LOG_INTERVAL_MS) {
    logData(currentTime, accelMagnitude, pressure, altitude);
    lastLogTime = currentTime;
  }
  
  // Print current status to serial (for debugging)
  if (currentTime - lastSerialPrintTime >= SERIAL_PRINT_INTERVAL) {
    Serial.print("Time: ");
    Serial.print(currentTime / 1000.0);
    Serial.print("s, Accel: ");
    Serial.print(accelMagnitude);
    Serial.print("g, Altitude: ");
    Serial.print(altitude);
    Serial.print("m, Pressure: ");
    Serial.print(pressure);
    Serial.print("Pa, Launch: ");
    Serial.print(launchDetected ? "YES" : "NO");
    Serial.print(", Servo: ");
    if (servoDeployed) {
      Serial.print("DEPLOYED");
    } else if (servoDeploymentInProgress) {
      Serial.print("DEPLOYING (");
      Serial.print(currentServoPos);
      Serial.print("°)");
    } else {
      Serial.print("WAITING (Alt: ");
      Serial.print(altitude);
      Serial.print("m vs ");
      Serial.print(DEPLOYMENT_ALTITUDE);
      Serial.print("m)");
    }
    Serial.println();
    
    lastSerialPrintTime = currentTime;
  }
}

void calibrateBMP() {
  // Discard initial readings
  for (int i = 0; i < BMP_WARMUP_READINGS; i++) {
    bmp.performReading();
    delay(100);
  }
  
  // Take multiple readings and average them for stability
  float altitudeSum = 0;
  int validReadings = 0;
  
  for (int i = 0; i < 10; i++) {
    if (bmp.performReading()) {
      // Use library's altitude calculation
      altitudeSum += bmp.readAltitude(1013.25);
      validReadings++;
    }
    delay(100);
  }
  
  if (validReadings > 0) {
    baselineAltitude = altitudeSum / validReadings;
    
    Serial.print("Baseline altitude: ");
    Serial.print(baselineAltitude);
    Serial.println(" m");
  } else {
    Serial.println("Calibration failed!");
    baselineAltitude = 0;
  }
}

void logData(unsigned long timestamp, float accelMagnitude, float pressure, float altitude) {
  if (dataFile) {
    dataFile.print(timestamp);
    dataFile.print(",");
    dataFile.print(accelMagnitude);
    dataFile.print(",");
    dataFile.print(pressure);
    dataFile.print(",");
    dataFile.print(altitude);
    dataFile.print(",");
    dataFile.print(launchDetected ? "1" : "0");
    dataFile.print(",");
    dataFile.print(servoDeployed ? "1" : "0");
    dataFile.print(",");
    dataFile.println(currentServoPos);
    dataFile.flush();
  }
}
