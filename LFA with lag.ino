//Throttle Signal → R1 (10kΩ) → ESP32 GPIO34 (ADC) → R2 (10kΩ) → GND
//(This scales 0.8-3.6V down to safe ADC range)

//ESP32 GPIO25 (or 26) → MAX98357A BCLK
//ESP32 GPIO22 (or 27) → MAX98357A LRC
//ESP32 GPIO21 → MAX98357A DIN
//MAX98357A OUT+ → Speaker +
//MAX98357A OUT- → Speaker -




#include <driver/i2s.h>

// Pin definitions
#define THROTTLE_PIN 34  // ADC pin for throttle input
#define I2S_BCLK 25
#define I2S_LRC 22
#define I2S_DOUT 21

// Audio settings
#define SAMPLE_RATE 44100
#define BUFFER_SIZE 512
#define I2S_NUM I2S_NUM_0

// LFA 1LR-GUE V10 Engine parameters
#define MIN_RPM 800
#define MAX_RPM 9500      // Rev limit
#define REDLINE 9000      // Where the magic happens
#define CYLINDERS 10      // V10 configuration
#define V_ANGLE 72.0      // 72-degree V10
#define THROTTLE_MIN 0.8  // Volts
#define THROTTLE_MAX 3.6  // Volts

// LFA-specific tuning
#define HIGH_RPM_THRESHOLD 6000  // Where the F1 scream starts
#define PEAK_POWER_RPM 8700      // Peak power point

// Throttle response tuning parameters
#define THROTTLE_RESPONSE_SPEED 0.08    // How fast throttle position changes (0.01=slow, 0.2=instant)
#define ENGINE_INERTIA_ACCEL 0.045      // How fast RPM rises (lower=more lag)
#define ENGINE_INERTIA_DECEL 0.035      // How fast RPM falls (lower=slower decel)
#define RPM_DAMPING 0.92                // Smoothing factor (0.9=smooth, 0.99=responsive)

// Global variables
float currentRPM = MIN_RPM;
float targetRPM = MIN_RPM;
float actualThrottle = 0.0;     // Current throttle position (with lag)
float targetThrottle = 0.0;     // Desired throttle position (from pedal)
float crankAngle = 0.0;
float intakePhase = 0.0;
float exhaustPhase = 0.0;
int16_t audioBuffer[BUFFER_SIZE];

// LFA firing order: 1-2-3-4-7-8-9-10-5-6
const int firingOrder[10] = {0, 1, 2, 3, 6, 7, 8, 9, 4, 5};
float cylinderPhases[10];

// Pre-calculated noise buffer
int8_t noiseBuffer[256];
int noiseIndex = 0;

void setup() {
  Serial.begin(115200);
  
  // Initialize cylinder phases for 72-degree V10
  for (int i = 0; i < 10; i++) {
    cylinderPhases[i] = firingOrder[i] * 72.0;
  }
  
  // Pre-generate noise buffer
  randomSeed(analogRead(0));
  for (int i = 0; i < 256; i++) {
    noiseBuffer[i] = random(-127, 128);
  }
  
  // Configure ADC
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);
  
  // Configure I2S
  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 8,
    .dma_buf_len = BUFFER_SIZE,
    .use_apll = false,
    .tx_desc_auto_clear = true,
    .fixed_mclk = 0
  };
  
  i2s_pin_config_t pin_config = {
    .bck_io_num = I2S_BCLK,
    .ws_io_num = I2S_LRC,
    .data_out_num = I2S_DOUT,
    .data_in_num = I2S_PIN_NO_CHANGE
  };
  
  i2s_driver_install(I2S_NUM, &i2s_config, 0, NULL);
  i2s_set_pin(I2S_NUM, &pin_config);
  i2s_zero_dma_buffer(I2S_NUM);
  
  Serial.println("Lexus LFA 1LR-GUE V10 Sound Simulator");
  Serial.println("72-degree V10 | 4.8L | 9000 RPM redline");
}

float readThrottle() {
  int adcValue = analogRead(THROTTLE_PIN);
  float voltage = (adcValue / 4095.0) * 3.3;
  
  float throttle = (voltage - THROTTLE_MIN) / (THROTTLE_MAX - THROTTLE_MIN);
  throttle = constrain(throttle, 0.0, 1.0);
  
  return throttle;
}

float mapThrottleToRPM(float throttle) {
  // LFA has a very linear, responsive throttle
  float rpm = MIN_RPM + pow(throttle, 1.3) * (MAX_RPM - MIN_RPM);
  return rpm;
}

// V10 cylinder firing with LFA characteristics
float lfaCylinderFiring(float angle, float rpm, float throttle) {
  float sample = 0.0;
  float fundamentalFreq = (rpm / 60.0);
  
  for (int cyl = 0; cyl < CYLINDERS; cyl++) {
    float cylAngle = fmod(angle - cylinderPhases[cyl] + 720.0, 720.0);
    
    // Firing pulse (every 72 degrees in a V10)
    if (cylAngle < 90.0) {
      float firePhase = cylAngle / 90.0;
      
      // Sharp, crisp combustion (LFA characteristic)
      float envelope = exp(-firePhase * 6.0);  // Faster decay = crisper
      
      float firingSample = 0.0;
      
      // More harmonics at high RPM for F1-like scream
      int maxHarmonic = (rpm > HIGH_RPM_THRESHOLD) ? 12 : 8;
      
      for (int h = 1; h <= maxHarmonic; h++) {
        float harmonic = sin(2.0 * PI * fundamentalFreq * h * firePhase);
        
        // Emphasize higher harmonics at high RPM (F1 scream)
        float harmAmp = 1.0 / h;
        if (rpm > HIGH_RPM_THRESHOLD && h > 4) {
          float highRPMBoost = (rpm - HIGH_RPM_THRESHOLD) / (MAX_RPM - HIGH_RPM_THRESHOLD);
          harmAmp *= (1.0 + highRPMBoost * 0.8);
        }
        
        firingSample += harmonic * harmAmp * envelope;
      }
      
      sample += firingSample * (0.3 + throttle * 0.7);
    }
  }
  
  return sample * 0.2;
}

// LFA's legendary intake sound
float lfaIntakeSound(float rpm, float throttle, float phase) {
  float intakeFreq = (rpm / 60.0) * (CYLINDERS / 2.0);
  float sample = 0.0;
  
  // The LFA intake is very prominent and clean
  for (int h = 1; h <= 10; h++) {
    float amp = 1.0 / (h * 1.5);
    sample += sin(2.0 * PI * intakeFreq * h * phase / SAMPLE_RATE) * amp;
  }
  
  // Clean air rush (not too much turbulence)
  float noise = (float)noiseBuffer[noiseIndex % 256] / 127.0;
  sample += noise * throttle * 0.2;
  
  // Intake becomes more prominent at high RPM
  float intakeVolume = 0.15;
  if (rpm > HIGH_RPM_THRESHOLD) {
    intakeVolume += (rpm - HIGH_RPM_THRESHOLD) / (MAX_RPM - HIGH_RPM_THRESHOLD) * 0.1;
  }
  
  return sample * throttle * intakeVolume;
}

// LFA exhaust - the star of the show
float lfaExhaustSound(float rpm, float throttle, float phase) {
  float exhaustFreq = (rpm / 60.0) * (CYLINDERS / 2.0);
  float sample = 0.0;
  
  // Rich harmonic content (what makes the LFA sound so good)
  int exhaustHarmonics = 16;
  if (rpm > HIGH_RPM_THRESHOLD) {
    exhaustHarmonics = 20;  // Even more harmonics at high RPM
  }
  
  for (int h = 1; h <= exhaustHarmonics; h++) {
    // LFA has very balanced harmonics - not too raspy, not too smooth
    float amp = 1.0 / sqrt(h * 1.2);
    
    // Boost mid-range harmonics for that screaming quality
    if (h >= 3 && h <= 8 && rpm > HIGH_RPM_THRESHOLD) {
      float screamBoost = (rpm - HIGH_RPM_THRESHOLD) / (MAX_RPM - HIGH_RPM_THRESHOLD);
      amp *= (1.0 + screamBoost * 0.6);
    }
    
    sample += sin(2.0 * PI * exhaustFreq * h * phase / SAMPLE_RATE) * amp;
  }
  
  // High-RPM exhaust crackle (LFA pops on overrun)
  if (rpm > 7000) {
    float crackle = (float)noiseBuffer[(noiseIndex * 5) % 256] / 127.0;
    float crackleIntensity = (rpm - 7000) / (MAX_RPM - 7000);
    
    // More crackle on decel (low throttle, high RPM)
    if (throttle < 0.3) {
      sample += crackle * crackleIntensity * 0.5;
    } else {
      sample += crackle * crackleIntensity * throttle * 0.3;
    }
  }
  
  // The LFA exhaust is very prominent
  float exhaustVolume = 0.35 + throttle * 0.5;
  
  // Peak volume around peak power RPM
  if (rpm > PEAK_POWER_RPM - 1000 && rpm < PEAK_POWER_RPM + 500) {
    exhaustVolume *= 1.15;
  }
  
  return sample * exhaustVolume;
}

// Mechanical sounds - very smooth for the LFA
float lfaMechanicalSound(float rpm) {
  float sample = 0.0;
  
  // Very refined valve train (barely audible)
  float valveFreq = (rpm / 60.0) * 2.0;
  sample += sin(2.0 * PI * valveFreq * crankAngle / 720.0) * 0.03;
  
  // Smooth crankshaft (well-balanced V10)
  float rumbleFreq = (rpm / 60.0) / 2.0;
  sample += sin(2.0 * PI * rumbleFreq * crankAngle / 720.0) * 0.08;
  
  return sample;
}

void generateLFASound() {
  // Read raw throttle input
  targetThrottle = readThrottle();
  
  // Apply throttle lag (simulates cable/electronic delay and airflow)
  float throttleDiff = targetThrottle - actualThrottle;
  actualThrottle += throttleDiff * THROTTLE_RESPONSE_SPEED;
  actualThrottle = constrain(actualThrottle, 0.0, 1.0);
  
  // Map throttle to target RPM
  targetRPM = mapThrottleToRPM(actualThrottle);
  
  // Calculate RPM change with different rates for accel/decel
  float rpmDiff = targetRPM - currentRPM;
  float acceleration;
  
  if (rpmDiff > 0) {
    // Accelerating - engine has to overcome inertia
    acceleration = rpmDiff * ENGINE_INERTIA_ACCEL;
    
    // LFA responds faster at high RPM (less rotating mass effect)
    if (currentRPM > HIGH_RPM_THRESHOLD) {
      acceleration *= 1.15;
    }
    
    // More throttle = faster response (more air/fuel)
    acceleration *= (0.6 + actualThrottle * 0.4);
  } else {
    // Decelerating - engine slows down due to friction and compression
    acceleration = rpmDiff * ENGINE_INERTIA_DECEL;
    
    // Engine braking is stronger at high RPM
    if (currentRPM > HIGH_RPM_THRESHOLD) {
      acceleration *= 1.2;
    }
  }
  
  // Apply RPM damping for smoother transitions
  currentRPM = currentRPM * RPM_DAMPING + (currentRPM + acceleration) * (1.0 - RPM_DAMPING);
  currentRPM = constrain(currentRPM, MIN_RPM, MAX_RPM);
  
  // 72-degree V10 crank rotation
  float degreesPerSample = (currentRPM / 60.0) * 720.0 / SAMPLE_RATE;
  
  for (int i = 0; i < BUFFER_SIZE; i++) {
    float sample = 0.0;
    
    // V10 cylinder firing
    sample += lfaCylinderFiring(crankAngle, currentRPM, actualThrottle);
    
    // LFA's iconic intake howl
    sample += lfaIntakeSound(currentRPM, actualThrottle, intakePhase);
    
    // The legendary exhaust note
    sample += lfaExhaustSound(currentRPM, actualThrottle, exhaustPhase);
    
    // Mechanical components (subtle)
    sample += lfaMechanicalSound(currentRPM);
    
    // Volume curve - LFA gets LOUD at high RPM
    float rpmVolume = 0.5 + (currentRPM - MIN_RPM) / (MAX_RPM - MIN_RPM) * 0.4;
    
    // Extra volume boost in the scream zone
    if (currentRPM > HIGH_RPM_THRESHOLD) {
      float screamZone = (currentRPM - HIGH_RPM_THRESHOLD) / (MAX_RPM - HIGH_RPM_THRESHOLD);
      rpmVolume *= (1.0 + screamZone * 0.3);
    }
    
    float throttleVolume = 0.4 + actualThrottle * 0.6;
    sample *= rpmVolume * throttleVolume;
    
    // Soft limiting
    sample = tanh(sample * 1.3);
    
    // Convert to 16-bit
    audioBuffer[i] = (int16_t)(sample * 32767.0 * 0.85);
    
    // Update phases
    crankAngle += degreesPerSample;
    if (crankAngle >= 720.0) crankAngle -= 720.0;
    
    intakePhase += 1.0;
    exhaustPhase += 1.0;
    noiseIndex++;
    
    if (intakePhase >= SAMPLE_RATE) intakePhase -= SAMPLE_RATE;
    if (exhaustPhase >= SAMPLE_RATE) exhaustPhase -= SAMPLE_RATE;
  }
  
  // Write to I2S
  size_t bytes_written;
  i2s_write(I2S_NUM, audioBuffer, BUFFER_SIZE * sizeof(int16_t), &bytes_written, portMAX_DELAY);
}

void loop() {
  generateLFASound();
  
  // Debug output
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 500) {
    Serial.print("Pedal: ");
    Serial.print(targetThrottle * 100, 1);
    Serial.print("% | Actual: ");
    Serial.print(actualThrottle * 100, 1);
    Serial.print("% | RPM: ");
    Serial.print(currentRPM, 0);
    Serial.print(" -> ");
    Serial.print(targetRPM, 0);
    
    // Show when in "scream zone"
    if (currentRPM > HIGH_RPM_THRESHOLD) {
      Serial.print(" [F1 MODE]");
    }
    if (currentRPM > REDLINE) {
      Serial.print(" [REDLINE!]");
    }
    
    Serial.println();
    lastPrint = millis();
  }
}
