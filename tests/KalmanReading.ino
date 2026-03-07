/*
 * Phototransistor Sensor Fusion Test
 * 
 * This standalone test file compares different approaches for combining
 * dual phototransistor readings:
 * 1. Simple averaging
 * 2. Sequential Kalman fusion
 * 3. Adaptive weighting
 * 
 * Upload this to your Arduino to test sensor fusion performance
 */

// Pin definitions - match your main code
#define LEFT_PHOTOTRANSISTOR_A A9
#define LEFT_PHOTOTRANSISTOR_B A11
#define RIGHT_PHOTOTRANSISTOR_A A10
#define RIGHT_PHOTOTRANSISTOR_B A12

// Test modes
enum TEST_MODE {
  COMPARE_METHODS,      // Compare all three methods side by side
  KALMAN_ONLY,         // Show only Kalman fusion details
  RAW_READINGS,        // Show raw sensor values
  STABILITY_TEST       // Test stability over time
};

TEST_MODE current_mode = COMPARE_METHODS;

// Kalman filter parameters
double process_noise_photo_left = 0.5;
double process_noise_photo_right = 0.5;
double sensor_noise_photo_left_A = 0.8;
double sensor_noise_photo_left_B = 0.8;
double sensor_noise_photo_right_A = 0.8;
double sensor_noise_photo_right_B = 0.8;

// State variables for Kalman filters
double leftPhotoVoltage_kalman = 0.0;
double rightPhotoVoltage_kalman = 0.0;
double last_var_photo_left = 1.0;
double last_var_photo_right = 1.0;

// Statistics tracking
struct SensorStats {
  double sum;
  double sum_squared;
  int count;
  double min_val;
  double max_val;
};

SensorStats left_simple_stats = {0, 0, 0, 999, -999};
SensorStats left_kalman_stats = {0, 0, 0, 999, -999};
SensorStats right_simple_stats = {0, 0, 0, 999, -999};
SensorStats right_kalman_stats = {0, 0, 0, 999, -999};

// Timing
unsigned long last_reading_time = 0;
unsigned long test_start_time = 0;
const int READING_INTERVAL = 100; // ms

// Function prototypes
double Kalman_sequential_dual(double measurement1, double measurement2, 
                             double prev_estimate, double prev_variance,
                             double sensor_noise1, double sensor_noise2,
                             double process_noise);
double Kalman_adaptive_weighting(double voltageA, double voltageB, 
                               double prev_estimate, double prev_variance,
                               double base_noise_A, double base_noise_B);
void updateStats(SensorStats* stats, double value);
void printStats(const char* name, SensorStats* stats);
void testCompareMethodsMode();
void testKalmanOnlyMode();
void testRawReadingsMode();
void testStabilityMode();
void processSerialCommands();

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10); // Wait for serial port
  
  Serial.println("=== Phototransistor Sensor Fusion Test ===");
  Serial.println("Commands:");
  Serial.println("  '1' - Compare Methods Mode");
  Serial.println("  '2' - Kalman Only Mode"); 
  Serial.println("  '3' - Raw Readings Mode");
  Serial.println("  '4' - Stability Test Mode");
  Serial.println("  's' - Show Statistics");
  Serial.println("  'r' - Reset Statistics");
  Serial.println("  'c' - Calibrate (set current as baseline)");
  Serial.println();
  
  // Initialize Kalman estimates
  leftPhotoVoltage_kalman = 2.5;   // Mid-range default
  rightPhotoVoltage_kalman = 2.5;
  last_var_photo_left = 1.0;
  last_var_photo_right = 1.0;
  
  test_start_time = millis();
  Serial.println("Starting in Compare Methods mode...");
}

void loop() {
  // Check for serial commands
  processSerialCommands();
  
  // Take readings at regular intervals
  if (millis() - last_reading_time >= READING_INTERVAL) {
    last_reading_time = millis();
    
    switch (current_mode) {
      case COMPARE_METHODS:
        testCompareMethodsMode();
        break;
      case KALMAN_ONLY:
        testKalmanOnlyMode();
        break;
      case RAW_READINGS:
        testRawReadingsMode();
        break;
      case STABILITY_TEST:
        testStabilityMode();
        break;
    }
  }
}

void processSerialCommands() {
  if (Serial.available()) {
    char cmd = Serial.read();
    switch (cmd) {
      case '1':
        current_mode = COMPARE_METHODS;
        Serial.println("\n=== Compare Methods Mode ===");
        break;
      case '2':
        current_mode = KALMAN_ONLY;
        Serial.println("\n=== Kalman Only Mode ===");
        break;
      case '3':
        current_mode = RAW_READINGS;
        Serial.println("\n=== Raw Readings Mode ===");
        break;
      case '4':
        current_mode = STABILITY_TEST;
        Serial.println("\n=== Stability Test Mode ===");
        break;
      case 's':
        Serial.println("\n=== STATISTICS ===");
        printStats("Left Simple", &left_simple_stats);
        printStats("Left Kalman", &left_kalman_stats);
        printStats("Right Simple", &right_simple_stats);
        printStats("Right Kalman", &right_kalman_stats);
        Serial.println();
        break;
      case 'r':
        Serial.println("Resetting statistics...");
        left_simple_stats = {0, 0, 0, 999, -999};
        left_kalman_stats = {0, 0, 0, 999, -999};
        right_simple_stats = {0, 0, 0, 999, -999};
        right_kalman_stats = {0, 0, 0, 999, -999};
        test_start_time = millis();
        break;
      case 'c':
        Serial.println("Calibrating - setting current readings as baseline...");
        // Reset Kalman filters to current readings
        leftPhotoVoltage_kalman = (analogRead(LEFT_PHOTOTRANSISTOR_A) + analogRead(LEFT_PHOTOTRANSISTOR_B)) * 5.0 / 2046.0;
        rightPhotoVoltage_kalman = (analogRead(RIGHT_PHOTOTRANSISTOR_A) + analogRead(RIGHT_PHOTOTRANSISTOR_B)) * 5.0 / 2046.0;
        last_var_photo_left = 1.0;
        last_var_photo_right = 1.0;
        break;
    }
  }
}

void testCompareMethodsMode() {
  // Read raw values
  int rawLeftA = analogRead(LEFT_PHOTOTRANSISTOR_A);
  int rawLeftB = analogRead(LEFT_PHOTOTRANSISTOR_B);
  int rawRightA = analogRead(RIGHT_PHOTOTRANSISTOR_A);
  int rawRightB = analogRead(RIGHT_PHOTOTRANSISTOR_B);
  
  // Convert to voltages
  float leftA = rawLeftA * 5.0 / 1023.0;
  float leftB = rawLeftB * 5.0 / 1023.0;
  float rightA = rawRightA * 5.0 / 1023.0;
  float rightB = rawRightB * 5.0 / 1023.0;
  
  // Method 1: Simple averaging
  double left_simple = (leftA + leftB) / 2.0;
  double right_simple = (rightA + rightB) / 2.0;
  
  // Method 2: Sequential Kalman fusion
  double left_kalman = Kalman_sequential_dual(leftA, leftB, leftPhotoVoltage_kalman, last_var_photo_left,
                                            sensor_noise_photo_left_A, sensor_noise_photo_left_B, process_noise_photo_left);
  double right_kalman = Kalman_sequential_dual(rightA, rightB, rightPhotoVoltage_kalman, last_var_photo_right,
                                             sensor_noise_photo_right_A, sensor_noise_photo_right_B, process_noise_photo_right);
  
  // Method 3: Adaptive weighting
  double left_adaptive = Kalman_adaptive_weighting(leftA, leftB, leftPhotoVoltage_kalman, last_var_photo_left,
                                                 sensor_noise_photo_left_A, sensor_noise_photo_left_B);
  double right_adaptive = Kalman_adaptive_weighting(rightA, rightB, rightPhotoVoltage_kalman, last_var_photo_right,
                                                  sensor_noise_photo_right_A, sensor_noise_photo_right_B);
  
  // Update Kalman states
  leftPhotoVoltage_kalman = left_kalman;
  rightPhotoVoltage_kalman = right_kalman;
  last_var_photo_left *= 0.8; // Decay variance
  last_var_photo_right *= 0.8;
  
  // Update statistics
  updateStats(&left_simple_stats, left_simple);
  updateStats(&left_kalman_stats, left_kalman);
  updateStats(&right_simple_stats, right_simple);
  updateStats(&right_kalman_stats, right_kalman);
  
  // Print comparison
  Serial.print("LEFT  - Raw: ");
  Serial.print(leftA, 2); Serial.print("/"); Serial.print(leftB, 2);
  Serial.print(" | Avg: "); Serial.print(left_simple, 3);
  Serial.print(" | Kal: "); Serial.print(left_kalman, 3);
  Serial.print(" | Adp: "); Serial.print(left_adaptive, 3);
  
  Serial.print(" || RIGHT - Raw: ");
  Serial.print(rightA, 2); Serial.print("/"); Serial.print(rightB, 2);
  Serial.print(" | Avg: "); Serial.print(right_simple, 3);
  Serial.print(" | Kal: "); Serial.print(right_kalman, 3);
  Serial.print(" | Adp: "); Serial.print(right_adaptive, 3);
  Serial.println();
}

void testKalmanOnlyMode() {
  // Read raw values
  float leftA = analogRead(LEFT_PHOTOTRANSISTOR_A) * 5.0 / 1023.0;
  float leftB = analogRead(LEFT_PHOTOTRANSISTOR_B) * 5.0 / 1023.0;
  float rightA = analogRead(RIGHT_PHOTOTRANSISTOR_A) * 5.0 / 1023.0;
  float rightB = analogRead(RIGHT_PHOTOTRANSISTOR_B) * 5.0 / 1023.0;
  
  // Apply Kalman fusion with detailed output
  Serial.print("L_in: "); Serial.print(leftA, 3); Serial.print(","); Serial.print(leftB, 3);
  Serial.print(" prev: "); Serial.print(leftPhotoVoltage_kalman, 3);
  Serial.print(" var: "); Serial.print(last_var_photo_left, 4);
  
  double left_result = Kalman_sequential_dual(leftA, leftB, leftPhotoVoltage_kalman, last_var_photo_left,
                                            sensor_noise_photo_left_A, sensor_noise_photo_left_B, process_noise_photo_left);
  
  Serial.print(" -> "); Serial.print(left_result, 3);
  Serial.print(" | R_in: "); Serial.print(rightA, 3); Serial.print(","); Serial.print(rightB, 3);
  
  double right_result = Kalman_sequential_dual(rightA, rightB, rightPhotoVoltage_kalman, last_var_photo_right,
                                             sensor_noise_photo_right_A, sensor_noise_photo_right_B, process_noise_photo_right);
  
  Serial.print(" -> "); Serial.println(right_result, 3);
  
  leftPhotoVoltage_kalman = left_result;
  rightPhotoVoltage_kalman = right_result;
  last_var_photo_left *= 0.9;
  last_var_photo_right *= 0.9;
}

void testRawReadingsMode() {
  // Just show raw ADC and voltage values
  int rawLeftA = analogRead(LEFT_PHOTOTRANSISTOR_A);
  int rawLeftB = analogRead(LEFT_PHOTOTRANSISTOR_B);
  int rawRightA = analogRead(RIGHT_PHOTOTRANSISTOR_A);
  int rawRightB = analogRead(RIGHT_PHOTOTRANSISTOR_B);
  
  Serial.print("LEFT_A: "); Serial.print(rawLeftA); Serial.print(" ("); Serial.print(rawLeftA * 5.0 / 1023.0, 3); Serial.print("V)");
  Serial.print(" | LEFT_B: "); Serial.print(rawLeftB); Serial.print(" ("); Serial.print(rawLeftB * 5.0 / 1023.0, 3); Serial.print("V)");
  Serial.print(" | RIGHT_A: "); Serial.print(rawRightA); Serial.print(" ("); Serial.print(rawRightA * 5.0 / 1023.0, 3); Serial.print("V)");
  Serial.print(" | RIGHT_B: "); Serial.print(rawRightB); Serial.print(" ("); Serial.print(rawRightB * 5.0 / 1023.0, 3); Serial.print("V)");
  Serial.println();
}

void testStabilityMode() {
  static int reading_count = 0;
  static double left_sum = 0, right_sum = 0;
  static double left_sq_sum = 0, right_sq_sum = 0;
  
  // Take readings
  float leftA = analogRead(LEFT_PHOTOTRANSISTOR_A) * 5.0 / 1023.0;
  float leftB = analogRead(LEFT_PHOTOTRANSISTOR_B) * 5.0 / 1023.0;
  float rightA = analogRead(RIGHT_PHOTOTRANSISTOR_A) * 5.0 / 1023.0;
  float rightB = analogRead(RIGHT_PHOTOTRANSISTOR_B) * 5.0 / 1023.0;
  
  double left_kalman = Kalman_sequential_dual(leftA, leftB, leftPhotoVoltage_kalman, last_var_photo_left,
                                            sensor_noise_photo_left_A, sensor_noise_photo_left_B, process_noise_photo_left);
  double right_kalman = Kalman_sequential_dual(rightA, rightB, rightPhotoVoltage_kalman, last_var_photo_right,
                                             sensor_noise_photo_right_A, sensor_noise_photo_right_B, process_noise_photo_right);
  
  leftPhotoVoltage_kalman = left_kalman;
  rightPhotoVoltage_kalman = right_kalman;
  last_var_photo_left *= 0.95;
  last_var_photo_right *= 0.95;
  
  // Accumulate statistics
  reading_count++;
  left_sum += left_kalman;
  right_sum += right_kalman;
  left_sq_sum += left_kalman * left_kalman;
  right_sq_sum += right_kalman * right_kalman;
  
  // Print running statistics every 10 readings
  if (reading_count % 10 == 0) {
    double left_mean = left_sum / reading_count;
    double right_mean = right_sum / reading_count;
    double left_variance = (left_sq_sum / reading_count) - (left_mean * left_mean);
    double right_variance = (right_sq_sum / reading_count) - (right_mean * right_mean);
    
    Serial.print("Readings: "); Serial.print(reading_count);
    Serial.print(" | Left: "); Serial.print(left_mean, 3); Serial.print(" ±"); Serial.print(sqrt(left_variance), 3);
    Serial.print(" | Right: "); Serial.print(right_mean, 3); Serial.print(" ±"); Serial.print(sqrt(right_variance), 3);
    Serial.println();
  }
}

// Sequential Kalman filter implementation
double Kalman_sequential_dual(double measurement1, double measurement2, 
                             double prev_estimate, double prev_variance,
                             double sensor_noise1, double sensor_noise2,
                             double process_noise) {
    
    // Prediction step
    double predicted_estimate = prev_estimate;
    double predicted_variance = prev_variance + process_noise;
    
    // Update step 1: Process first sensor
    double kalman_gain1 = predicted_variance / (predicted_variance + sensor_noise1);
    double updated_estimate1 = predicted_estimate + kalman_gain1 * (measurement1 - predicted_estimate);
    double updated_variance1 = (1.0 - kalman_gain1) * predicted_variance;
    
    // Update step 2: Process second sensor
    double kalman_gain2 = updated_variance1 / (updated_variance1 + sensor_noise2);
    double final_estimate = updated_estimate1 + kalman_gain2 * (measurement2 - updated_estimate1);
    
    return final_estimate;
}

// Adaptive weighting implementation
double Kalman_adaptive_weighting(double voltageA, double voltageB, 
                               double prev_estimate, double prev_variance,
                               double base_noise_A, double base_noise_B) {
    
    // Adaptive noise based on how realistic the readings are
    double adaptive_noise_A = base_noise_A;
    double adaptive_noise_B = base_noise_B;
    
    // Increase noise for readings that seem unrealistic
    if (voltageA < 0.05 || voltageA > 4.5) {
        adaptive_noise_A *= 3.0;
    }
    if (voltageB < 0.05 || voltageB > 4.5) {
        adaptive_noise_B *= 3.0;
    }
    
    // Also consider how far from previous estimate
    double error_A = abs(voltageA - prev_estimate);
    double error_B = abs(voltageB - prev_estimate);
    
    if (error_A > 1.0) adaptive_noise_A *= 2.0;
    if (error_B > 1.0) adaptive_noise_B *= 2.0;
    
    return Kalman_sequential_dual(voltageA, voltageB, prev_estimate, prev_variance,
                                 adaptive_noise_A, adaptive_noise_B, 0.1);
}

// Statistics helper functions
void updateStats(SensorStats* stats, double value) {
  stats->sum += value;
  stats->sum_squared += value * value;
  stats->count++;
  if (value < stats->min_val) stats->min_val = value;
  if (value > stats->max_val) stats->max_val = value;
}

void printStats(const char* name, SensorStats* stats) {
  if (stats->count == 0) {
    Serial.print(name); Serial.println(": No data");
    return;
  }
  
  double mean = stats->sum / stats->count;
  double variance = (stats->sum_squared / stats->count) - (mean * mean);
  double std_dev = sqrt(variance);
  
  Serial.print(name); Serial.print(": ");
  Serial.print("Mean="); Serial.print(mean, 3);
  Serial.print(" StdDev="); Serial.print(std_dev, 3);
  Serial.print(" Min="); Serial.print(stats->min_val, 3);
  Serial.print(" Max="); Serial.print(stats->max_val, 3);
  Serial.print(" (n="); Serial.print(stats->count); Serial.println(")");
}
