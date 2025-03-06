// Team Member: Andy Deng yd2580, Ruichen Gao rg4238, Lan Zhang lz2620 

#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_NeoPixel.h>

// ********** LED Configuration **********
#define NEOPIXEL_PIN     17
#define NUMPIXELS        10
Adafruit_NeoPixel pixels(NUMPIXELS, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);

// Define button pins
#define BUTTON_RECORD 4     // Button to record gesture
#define BUTTON_UNLOCK 19    // Button to unlock device

#define DATA_LENGTH 60      // Data length / 20 = gesture time in seconds
#define TOLERANCE 500000

int16_t recorded_data [DATA_LENGTH][3];     // Data of recorded gesture
int16_t captured_data [DATA_LENGTH][3];     // Data of captured gesture

// Variables for record() LED indication
bool processStarted = false;
int illuminatedLEDs = 0;
unsigned long previousMillis = 0;
const unsigned long interval = 1000; // 1 second per LED pair
static unsigned long finishedTime = 0;
static bool finishedStage = false; // Indicates we've lit all LED pairs and are waiting

// Function prototypes
void get_data(int16_t data[DATA_LENGTH][3]);
int8_t compare_sequence(void);
void read_accelerometer(int16_t *x, int16_t *y, int16_t *z);

// LED helper functions
void clearLEDs();
void correct();
void wrong();
void recordIndicator();  // Renamed record() to recordIndicator() to avoid confusion with the gesture recording action



void setup() {
    pinMode(BUTTON_RECORD, INPUT);
    pinMode(BUTTON_UNLOCK, INPUT);

    // Initialize Accelerometer SPI interface
    DDRB |= (1 << PIN4); // PIN4 as CS output
    SPI.begin();

    // Configure accelerometer CTRL_REG1
    PORTB &= ~(1 << PIN4);
    SPI.transfer(0x20);
    SPI.transfer(0x37); // Enable all axes, 20Hz data rate
    PORTB |= (1 << PIN4);

    // Configure accelerometer CTRL_REG4
    PORTB &= ~(1 << PIN4);
    SPI.transfer(0x23);
    SPI.transfer(0x00); // ±2g range, continuous update
    PORTB |= (1 << PIN4);

    // Initialize NeoPixels
    pixels.begin();
    clearLEDs();

    Serial.begin(115200);
}


void loop() {
    // Check if the record button is pressed
    if (digitalRead(BUTTON_RECORD) == HIGH) {
        Serial.println("Recording started...");

        // Start LED recording indicator
        processStarted = true;
        illuminatedLEDs = 0;
        finishedStage = false;
        previousMillis = millis();
        finishedTime = 0;

        // Record accelerometer data (3 seconds approx)
        get_data(recorded_data);

        Serial.println("Recording finished.");
        processStarted = false;  // Stop LED recording indicator

        // At this point, recordIndicator() would have displayed the LED pattern
        // for the duration of the recording. After finishing, it should clear automatically.
        
    } else if (digitalRead(BUTTON_UNLOCK) == HIGH) {
        Serial.println("Capture started...");

        // Just clear LEDs before capture (optional)
        clearLEDs();

        get_data(captured_data);
        
        Serial.println("Capture finished.");

        int8_t result = compare_sequence();
        Serial.println("Compare Done");

        if (result) {
            Serial.println("Match Found: Unlock Successful");
            correct();  
        } else {
            Serial.println("No Match: Unlock Failed");
            wrong();
        }
    }

    delay(100); // Avoid button bouncing
}

int8_t compare_sequence(void) {

    Serial.println("Compare Start");

    int32_t dtw_prev[DATA_LENGTH];
    int32_t dtw_curr[DATA_LENGTH];

    // Initialize first row
    for (int j = 0; j < DATA_LENGTH; j++) {
        dtw_prev[j] = (j == 0) ? 0 : INT32_MAX;
    }

    // Compute DTW
    for (int i = 0; i < DATA_LENGTH; i++) {
        dtw_curr[0] = (i == 0) ? 0 : INT32_MAX;

        for (int j = 1; j < DATA_LENGTH; j++) {
            int32_t cost = 0;
            for (int k = 0; k < 3; k++) { // X, Y, Z axes
                cost += abs(recorded_data[i][k] - captured_data[j][k]);
            }
            // Serial.print("Cost: ");
            // Serial.println(cost);
            dtw_curr[j] = cost + min(min(dtw_prev[j], dtw_curr[j - 1]), dtw_prev[j - 1]);
            // Serial.print("dtw_curr: ");
            // Serial.println(dtw_curr[j]);
        }

        // Copy current row to previous row
        memcpy(dtw_prev, dtw_curr, sizeof(dtw_curr));
    }

    // Get DTW distance
    int32_t dtw_distance = dtw_prev[DATA_LENGTH - 1];
    Serial.print("DTW Distance: ");
    Serial.println(dtw_distance);
    return (dtw_distance < TOLERANCE) ? 1 : 0;
}



void read_accelerometer(int16_t *x, int16_t *y, int16_t *z) {

    uint8_t data_buffer[6];

    // Set chip select low to initiate communication
    PORTB &= ~(1 << PIN4);

    // Send the address of OUT_X_L (starting address for X, Y, Z data) with auto-increment enabled
    SPI.transfer(0x28 | (1 << 7) | (1 << 6)); // 0x28 = OUT_X_L, 0x80 for read, 0x40 for auto-increment

    // Read 6 bytes sequentially: X_L, X_H, Y_L, Y_H, Z_L, Z_H
    for (int i = 0; i < 6; i++) {
        data_buffer[i] = SPI.transfer(0xFF); // Send dummy byte to retrieve data
    }

    // Set chip select high to end communication
    PORTB |= (1 << PIN4);

    // Combine high and low bytes for each axis and store as signed 16-bit integers
    *x = (int16_t)(data_buffer[1] << 8 | data_buffer[0]); // X-axis
    *y = (int16_t)(data_buffer[3] << 8 | data_buffer[2]); // Y-axis
    *z = (int16_t)(data_buffer[5] << 8 | data_buffer[4]); // Z-axis
}


void get_data(int16_t data[DATA_LENGTH][3]) {
    int16_t x, y, z;
    int8_t illuminatedLEDs = 0;
    for (int i = 0; i < DATA_LENGTH; i++) {
        // Read X, Y, Z values from the accelerometer
        read_accelerometer(&x, &y, &z); // Use pointers to store the values in variables

        // Store the read values into sequance
        data[i][0] = x; // X-axis
        data[i][1] = y; // Y-axis
        data[i][2] = z; // Z-axis
        if(i % 20 == 0) {
            pixels.setPixelColor(illuminatedLEDs, pixels.Color(25, 105, 25));
            pixels.setPixelColor((NUMPIXELS - 1) - illuminatedLEDs, pixels.Color(25, 105, 25));
            pixels.show();
            illuminatedLEDs++;
        }

        // Wait 50 ms before the next sample to maintain a 20 Hz sampling rate
        delay(50); // Delay for 50 ms
    }
    clearLEDs();
}



// LED Functions
void clearLEDs() {
    pixels.clear();
    pixels.show();
}

void correct() {
    // Set LEDs green
    for (int i = 0; i < NUMPIXELS; i++) {
        pixels.setPixelColor(i, pixels.Color(15, 135, 15)); // Green
    }
    pixels.show();
    delay(10000);
    clearLEDs();
}

void wrong() {
    // Set LEDs red
    for (int i = 0; i < NUMPIXELS; i++) {
        pixels.setPixelColor(i, pixels.Color(135, 15, 15)); // Red
    }
    pixels.show();
    delay(3000);
    clearLEDs();
}
