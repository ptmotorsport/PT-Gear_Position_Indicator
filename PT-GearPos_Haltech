/// @file    PT-GearPos.ino
/// @brief   Gear Position Indicator with 5x5 NeoPixel Matrix and CAN Bus
/// @description Displays gear positions from Haltech CAN data (0x470 byte 7) (Haltech)

#include <FastLED.h>
#include <mcp_can.h>
#include <SPI.h>

// CAN Bus Configuration
#define CAN_CS_PIN 10           // CS pin for MCP2515
#define CAN_SPEED CAN_1000KBPS  // 1Mbps CAN speed
#define CAN_CLOCK MCP_8MHZ      // 8MHz crystal

MCP_CAN CAN(CAN_CS_PIN);        // Set CS pin

// CAN Message IDs
#define HALTECH_GEAR_ID 0x470
#define BLINK_LED_ID 0x215      // Blink Marine PKP 2500SI LED control message

// LED Matrix Configuration
#define DATA_PIN    7
#define LED_TYPE    WS2811
#define COLOR_ORDER GRB
#define NUM_LEDS    25
CRGB leds[NUM_LEDS];

#define BRIGHTNESS          255 // choose brightness from 0-255

// LED Layout for 5x5 Matrix (25 LEDs total)
// LEDs 0-4:   Status lights (top row - reserved)
// LEDs 5-19:  Character display (middle 3 rows, 3x5 grid for numbers)
// LEDs 20-24: Status lights (bottom row - reserved)

// Display area is LEDs 5-19 (15 LEDs in middle 3 rows)
// Using a 3x5 pixel font for characters

// Color definitions
#define COLOR_GEAR CRGB::Magenta
#define COLOR_NEUTRAL CRGB::Yellow
#define COLOR_REVERSE CRGB::Red

// 5x3 Character patterns for the middle 15 LEDs (rows 1-3, all 5 columns)
// Each character is defined as a 5x3 bitmap
// 1 = LED on, 0 = LED off
// Organized as rows from top to bottom, columns left to right

// Character 'R' (Reverse)
const bool CHAR_R[15] = {
  1,1,1,0,0,  // Row 1 (top)
  0,0,1,0,0,  // Row 2 (middle)
  0,0,1,0,0   // Row 3 (bottom)
};

// Character 'N' (Neutral)
const bool CHAR_N[15] = {
  1,1,1,0,0,
  0,0,1,0,0,
  1,1,1,0,0
};

// Character '1'
const bool CHAR_1[15] = {
  0,0,0,0,0,
  0,0,0,0,0,
  1,1,1,1,1
};

// Character '2'
const bool CHAR_2[15] = {
  1,1,1,0,1,
  1,0,1,0,1,
  1,0,1,1,1
};

// Character '3'
const bool CHAR_3[15] = {
  1,0,1,0,1,
  1,0,1,0,1,
  1,1,1,1,1
};

// Character '4'
const bool CHAR_4[15] = {
  0,0,1,1,1,
  0,0,1,0,0,
  1,1,1,1,1
};

// Character '5'
const bool CHAR_5[15] = {
  1,0,1,1,1,
  1,0,1,0,1,
  1,1,1,0,1
};

// Character '6'
const bool CHAR_6[15] = {
  1,1,1,1,1,
  1,0,1,0,1,
  1,1,1,0,1
};

// Character '7'
const bool CHAR_7[15] = {
  0,0,0,0,1,
  0,0,0,0,1,
  1,1,1,1,1
};

// Character '8'
const bool CHAR_8[15] = {
  1,1,1,1,1,
  1,0,1,0,1,
  1,1,1,1,1
};

// Character '9'
const bool CHAR_9[15] = {
  1,0,0,1,1,
  1,0,1,0,1,
  1,1,1,1,1
};

// Character 'O' (Other)
const bool CHAR_O[15] = {
  1,1,1,1,1,
  1,0,0,0,1,
  1,1,1,1,1
};

// Character 'L' (Low)
const bool CHAR_L[15] = {
  1,1,1,1,1,
  1,0,0,0,0,
  1,0,0,0,0
};

// Character 'M' (Manual)
const bool CHAR_M[15] = {
  1,1,0,1,1,
  1,0,1,0,1,
  1,0,1,0,1
};

// Character 'S' (Sport)
const bool CHAR_S[15] = {
  1,0,1,1,1,
  1,0,1,0,1,
  1,1,1,0,1
};

// Character 'D' (Drive)
const bool CHAR_D[15] = {
  1,1,1,0,0,
  1,0,1,0,0,
  1,1,1,1,1
};

// Character 'U' (Unknown)
const bool CHAR_U[15] = {
  1,1,1,1,1,
  1,0,0,0,0,
  1,1,1,1,1
};

// Character 'P' (Park)
const bool CHAR_P[15] = {
  1,1,1,1,1,
  0,0,1,0,1,
  0,0,1,1,1
};

// Current gear state
byte currentGearByte = 0;    // Raw gear byte from CAN
char currentGearChar = 'N';   // Decoded gear character
unsigned long lastCANMsg = 0; // Timestamp of last CAN message

// Status LED state (Blink Marine protocol extended)
uint16_t statusRedLEDs = 0;      // Bytes 0+1: Red LED states (bits 0-9 for 10 LEDs)
uint16_t statusGreenLEDs = 0;    // Bytes 2+3: Green LED states (bits 0-9)
uint16_t statusBlueLEDs = 0;     // Bytes 4+5: Blue LED states (bits 0-9)
unsigned long lastStatusMsg = 0; // Timestamp of last status message

// Display update timing (15Hz)
unsigned long lastDisplayUpdate = 0;
const unsigned long DISPLAY_UPDATE_INTERVAL = 67; // ~15Hz (1000ms / 15 = 66.67ms)

// Variable for rainbow effect
uint8_t gHue = 0;

// Helper function to convert logical grid position to physical LED index
// Accounts for serpentine wiring pattern
// col: 0=left, 1=middle, 2=right (from viewer perspective)
// row: 0=top, 4=bottom
int getLEDIndex(int row, int col) {
  // Row 0 (top): LEDs 0-4, RIGHT to LEFT (4,3,2,1,0)
  // Row 1: LEDs 5-9, LEFT to RIGHT (5,6,7,8,9)
  // Row 2: LEDs 10-14, RIGHT to LEFT (14,13,12,11,10)
  // Row 3: LEDs 15-19, LEFT to RIGHT (15,16,17,18,19)
  // Row 4 (bottom): LEDs 20-24, RIGHT to LEFT (24,23,22,21,20)
  
  int rowStart = row * 5;
  
  if (row % 2 == 0) {
    // Even rows (0,2,4): RIGHT to LEFT, so reverse the column
    return rowStart + (4 - col);
  } else {
    // Odd rows (1,3): LEFT to RIGHT, normal order
    return rowStart + col;
  }
}

void setup() {
  delay(1000); // Short delay for recovery
  
  // Configure FastLED
  FastLED.addLeds<LED_TYPE, DATA_PIN, COLOR_ORDER>(leds, NUM_LEDS).setCorrection(TypicalLEDStrip);
  FastLED.setBrightness(BRIGHTNESS);
  
  // Clear all LEDs
  FastLED.clear();
  FastLED.show();
  
  // Initialize CAN Bus
  while (CAN.begin(MCP_ANY, CAN_SPEED, CAN_CLOCK) != CAN_OK) {
    // Flash top row red if CAN init fails
    for(int i = 0; i < 5; i++) {
      leds[i] = CRGB::Red;
    }
    FastLED.show();
    delay(500);
    FastLED.clear();
    FastLED.show();
    delay(500);
  }
  
  // Configure CAN filters to only receive gear and LED status messages
  // Filter 0: Accept 0x470 (Haltech Gear)
  CAN.init_Mask(0, 0, 0x7FF);                    // Mask 0: Check all 11 bits
  CAN.init_Filt(0, 0, HALTECH_GEAR_ID);          // Filter 0: Accept 0x470
  
  // Filter 1: Accept 0x215 (Blink LED)
  CAN.init_Filt(1, 0, BLINK_LED_ID);             // Filter 1: Accept 0x215
  
  // Remaining filters: reject all (set to non-existent IDs)
  CAN.init_Mask(1, 0, 0x7FF);                    // Mask 1: Check all 11 bits
  CAN.init_Filt(2, 0, 0x7FF);                    // Filter 2-5: Reject
  CAN.init_Filt(3, 0, 0x7FF);
  CAN.init_Filt(4, 0, 0x7FF);
  CAN.init_Filt(5, 0, 0x7FF);
  
  // Set CAN to normal mode
  CAN.setMode(MCP_NORMAL);
  
  // Flash top row green to indicate CAN init success
  for(int i = 0; i < 5; i++) {
    leds[i] = CRGB::Green;
  }
  FastLED.show();
  delay(500);
  FastLED.clear();
  FastLED.show();
  
  // Startup animation: Sinelon on status LEDs for 2 seconds
  rainbowChaseStartup();
}
  
void loop() {
  // Read CAN messages (always process incoming messages)
  readCANMessages();
  
  // Update display at 15Hz to reduce CPU load
  if (millis() - lastDisplayUpdate >= DISPLAY_UPDATE_INTERVAL) {
    lastDisplayUpdate = millis();
    
    // Display current gear (this clears only the middle area)
    displayGear(currentGearChar);
    
    // Update status LEDs AFTER gear display so they don't get cleared
    updateStatusLEDs();
    
    // Send to LEDs
    FastLED.show();
  }
  
  // Check for CAN timeout (no message for 2 seconds)
  if (millis() - lastCANMsg > 2000 && lastCANMsg > 0) {
    // Flash display to indicate CAN timeout
    // Could add error indication here
  }
}

// Read CAN messages and decode gear position
void readCANMessages() {
  long unsigned int rxId;
  unsigned char len = 0;
  unsigned char rxBuf[8];
  
  // Check if data is available
  if (CAN.checkReceive() == CAN_MSGAVAIL) {
    CAN.readMsgBuf(&rxId, &len, rxBuf);
    
    // Check if this is the gear position message
    if (rxId == HALTECH_GEAR_ID && len >= 8) {
      currentGearByte = rxBuf[7];  // Byte 7 contains gear position
      lastCANMsg = millis();
      
      // Decode gear byte to character
      switch(currentGearByte) {
        case 0:
          currentGearChar = 'N';
          break;
        case 1:
          currentGearChar = '1';
          break;
        case 2:
          currentGearChar = '2';
          break;
        case 3:
          currentGearChar = '3';
          break;
        case 4:
          currentGearChar = '4';
          break;
        case 5:
          currentGearChar = '5';
          break;
        case 6:
          currentGearChar = '6';
          break;
        case 7:
          currentGearChar = '7';
          break;
        case 8:
          currentGearChar = '8';
          break;
        case 9:
          currentGearChar = '9';
          break;
        case 248: // 0xF8
          currentGearChar = 'O';
          break;
        case 249: // 0xF9
          currentGearChar = 'L';
          break;
        case 250: // 0xFA
          currentGearChar = 'M';
          break;
        case 251: // 0xFB
          currentGearChar = 'S';
          break;
        case 252: // 0xFC
          currentGearChar = 'D';
          break;
        case 253: // 0xFD
          currentGearChar = 'U';
          break;
        case 254: // 0xFE
          currentGearChar = 'P';
          break;
        case 255: // 0xFF
          currentGearChar = 'R';
          break;
        default:
          currentGearChar = 'U'; // Unknown
          break;
      }
    }
    // Check if this is the Blink Marine LED control message
    else if (rxId == BLINK_LED_ID && len >= 6) {
      // Byte 0: Red LEDs 1-8 (bits 0-7)
      // Byte 1: Red LEDs 9-10 (bits 0-1)
      statusRedLEDs = rxBuf[0] | ((rxBuf[1] & 0x03) << 8);
      
      // Byte 2: Green LEDs 1-8 (bits 0-7)
      // Byte 3: Green LEDs 9-10 (bits 0-1)
      statusGreenLEDs = rxBuf[2] | ((rxBuf[3] & 0x03) << 8);
      
      // Byte 4: Blue LEDs 1-8 (bits 0-7)
      // Byte 5: Blue LEDs 9-10 (bits 0-1)
      statusBlueLEDs = rxBuf[4] | ((rxBuf[5] & 0x03) << 8);
      
      lastStatusMsg = millis();
      lastCANMsg = millis();
    }
  }
}

// Update status LEDs based on Blink Marine protocol (extended to 10 LEDs)
void updateStatusLEDs() {
  // Extended Blink Marine protocol for 10 LEDs:
  // Byte 0 + Byte 1 bit 0: Red LEDs 1-9 (bits 0-8)
  // Byte 1 bit 1: Red LED 10 (bit 9)
  // Same pattern for Green (bytes 2+3) and Blue (bytes 4+5)
  //
  // Top row: LEDs 0-4 (bits 0-4)
  // Bottom row: LEDs 20-24 (bits 5-9)
  
  for(int i = 0; i < 5; i++) {
    // Top row status LEDs
    int topLED = getLEDIndex(0, i);
    bool redOn = (statusRedLEDs >> i) & 0x01;
    bool greenOn = (statusGreenLEDs >> i) & 0x01;
    bool blueOn = (statusBlueLEDs >> i) & 0x01;
    
    leds[topLED] = CRGB(redOn ? 255 : 0, greenOn ? 255 : 0, blueOn ? 255 : 0);
    
    // Bottom row status LEDs
    int bottomLED = getLEDIndex(4, i);
    bool redOn2 = (statusRedLEDs >> (i + 5)) & 0x01;
    bool greenOn2 = (statusGreenLEDs >> (i + 5)) & 0x01;
    bool blueOn2 = (statusBlueLEDs >> (i + 5)) & 0x01;
    
    leds[bottomLED] = CRGB(redOn2 ? 255 : 0, greenOn2 ? 255 : 0, blueOn2 ? 255 : 0);
  }
}

// Function to display a character on the middle 15 LEDs
void displayGear(char gear) {
  const bool* pattern = nullptr;
  CRGB color;
  
  // Select the pattern and color based on gear
  switch(gear) {
    case 'R':
      pattern = CHAR_R;
      color = COLOR_REVERSE;
      break;
    case 'N':
      pattern = CHAR_N;
      color = COLOR_NEUTRAL;
      break;
    case '1':
      pattern = CHAR_1;
      color = COLOR_GEAR;
      break;
    case '2':
      pattern = CHAR_2;
      color = COLOR_GEAR;
      break;
    case '3':
      pattern = CHAR_3;
      color = COLOR_GEAR;
      break;
    case '4':
      pattern = CHAR_4;
      color = COLOR_GEAR;
      break;
    case '5':
      pattern = CHAR_5;
      color = COLOR_GEAR;
      break;
    case '6':
      pattern = CHAR_6;
      color = COLOR_GEAR;
      break;
    case '7':
      pattern = CHAR_7;
      color = COLOR_GEAR;
      break;
    case '8':
      pattern = CHAR_8;
      color = COLOR_GEAR;
      break;
    case '9':
      pattern = CHAR_9;
      color = COLOR_GEAR;
      break;
    case 'O':
      pattern = CHAR_O;
      color = CRGB::Orange;
      break;
    case 'L':
      pattern = CHAR_L;
      color = CRGB::Blue;
      break;
    case 'M':
      pattern = CHAR_M;
      color = CRGB::Cyan;
      break;
    case 'S':
      pattern = CHAR_S;
      color = CRGB::Purple;
      break;
    case 'D':
      pattern = CHAR_D;
      color = CRGB::Green;
      break;
    case 'U':
      pattern = CHAR_U;
      color = CRGB::White;
      break;
    case 'P':
      pattern = CHAR_P;
      color = CRGB::Yellow;
      break;
    default:
      clearDisplay();
      return;
  }
  
  // Clear all LEDs first
  clearDisplay();
  
  // Set the middle 15 LEDs (rows 1-3 of the 5x5 matrix, all 5 columns)
  // Pattern is defined as 5 columns x 3 rows
  for(int row = 0; row < 3; row++) {
    for(int col = 0; col < 5; col++) {
      int patternIndex = row * 5 + col;  // Index into the pattern array
      if(pattern[patternIndex]) {
        // Map to physical LED: use rows 1-3 (middle rows), cols 0-4 (all columns)
        int ledIndex = getLEDIndex(row + 1, col);
        leds[ledIndex] = color;
      }
    }
  }
}

// Clear only the display area (middle 15 LEDs), preserve status LEDs
void clearDisplay() {
  // Only clear the middle 3 rows (LEDs 5-19)
  for(int row = 1; row < 4; row++) {
    for(int col = 0; col < 5; col++) {
      int ledIndex = getLEDIndex(row, col);
      leds[ledIndex] = CRGB::Black;
    }
  }
}

// Sinelon animation on status LEDs (rows 0 and 4) - sweeping dot with fading trails
void rainbowChaseStartup() {
  unsigned long startTime = millis();
  
  // Create array of just the status LED indices (top and bottom rows)
  int statusLEDs[10] = {0, 1, 2, 3, 4, 20, 21, 22, 23, 24};
  
  // Run for 2 seconds
  while(millis() - startTime < 2900) {
    // Fade status LEDs
    for(int i = 0; i < 10; i++) {
      leds[statusLEDs[i]].fadeToBlackBy(20);
    }
    
    // Create sweeping dot using beatsin16
    int pos = beatsin16(13, 0, 9);  // Sweep across 10 status LEDs
    leds[statusLEDs[pos]] += CHSV(gHue, 255, 192);
    
    FastLED.show();
    gHue += 2;  // Rotate through rainbow colors
    delay(20);
  }
  
  // Clear status LEDs after animation
  for(int i = 0; i < 10; i++) {
    leds[statusLEDs[i]] = CRGB::Black;
  }
  FastLED.show();
}
