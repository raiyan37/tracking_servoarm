#include <Servo.h>

Servo servos[5];  // Array for cleaner code
const byte SERVO_PINS[5] = {2, 3, 4, 5, 6};

// Use char array instead of String (much more efficient)
char inputBuffer[32];  // Fixed size buffer
byte bufferIndex = 0;

void setup() {
  Serial.begin(9600);  // Match Python baud rate
  
  // Attach servos
  for (byte i = 0; i < 5; i++) {
    servos[i].attach(SERVO_PINS[i]);
  }
  
  Serial.println("READY");
}

void loop() {
  while (Serial.available()) {
    char c = Serial.read();
    
    if (c == '\n') {
      inputBuffer[bufferIndex] = '\0';  // Null terminate
      processCommand();
      bufferIndex = 0;  // Reset buffer
    } 
    else if (bufferIndex < sizeof(inputBuffer) - 1) {  // Prevent overflow
      inputBuffer[bufferIndex++] = c;
    }
    // If buffer full, ignore additional chars until newline
  }
}

void processCommand() {
  int values[5];
  byte index = 0;
  
  // Parse comma-separated values
  char* token = strtok(inputBuffer, ",");
  
  while (token != NULL && index < 5) {
    values[index++] = atoi(token);
    token = strtok(NULL, ",");
  }
  
  // Only update if we received all 5 values
  if (index == 5) {
    for (byte i = 0; i < 5; i++) {
      servos[i].write(constrain(values[i], 0, 180));
    }
  }
}