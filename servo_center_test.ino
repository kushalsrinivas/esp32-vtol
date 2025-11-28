/* SERVO CENTER CALIBRATION TOOL
   
   This code helps you find the TRUE center position of your servos.
   The servo will hold at 90° (dead center position) so you can:
   1. Check if the servo arm is physically centered
   2. Adjust the servo arm position if needed
   3. Determine if you need to offset the center value in your main code
   
   HARDWARE SETUP:
   ===============
   RIGHT ELEVON:
     Signal -> GPIO 25
     VCC    -> 5V (external power supply recommended)
     GND    -> GND
   
   LEFT ELEVON:
     Signal -> GPIO 26
     VCC    -> 5V (external power supply)
     GND    -> GND
   
   INSTRUCTIONS:
   =============
   1. Upload this code to your ESP32
   2. Open Serial Monitor (115200 baud)
   3. Observe both servos - they should be at dead center (90°)
   4. Check if the control arms/horns are perpendicular to the servo body
   5. If not centered:
      - Remove servo horn, rotate to center position, reattach
      - OR note the true center angle and use it in your main code
   
   SERIAL COMMANDS:
   ================
   Send these commands via Serial Monitor:
   - Type a number (0-180) and press Enter to test that angle
   - Type 'r' to move RIGHT servo only
   - Type 'l' to move LEFT servo only
   - Type 'b' to move BOTH servos (default)
   - Type 'c' to return to CENTER (90°)
   
   Examples:
   - "90" -> both servos to 90°
   - "45" -> both servos to 45°
   - "r" then "90" -> right servo to 90°
   - "l" then "120" -> left servo to 120°
*/

#include <ESP32Servo.h>

// Pin definitions
#define RIGHT_ELEVON_PIN 25
#define LEFT_ELEVON_PIN 26

// Center position
#define CENTER 90

// Servo objects
Servo right_elevon;
Servo left_elevon;

// Control variables
enum ServoControl { BOTH, RIGHT_ONLY, LEFT_ONLY };
ServoControl servoMode = BOTH;
int currentAngle = CENTER;

void setup() {
    Serial.begin(115200);
    delay(2000);
    
    Serial.println("\n╔═══════════════════════════════════════╗");
    Serial.println("║   SERVO CENTER CALIBRATION TOOL      ║");
    Serial.println("╚═══════════════════════════════════════╝\n");
    
    // Attach servos
    Serial.println("→ Attaching servos...");
    right_elevon.attach(RIGHT_ELEVON_PIN);
    left_elevon.attach(LEFT_ELEVON_PIN);
    delay(100);
    
    // Center both servos
    Serial.println("→ Centering servos to 90°...");
    right_elevon.write(CENTER);
    left_elevon.write(CENTER);
    delay(500);
    
    Serial.println("✓ Servos centered!\n");
    Serial.println("╔═══════════════════════════════════════╗");
    Serial.println("║  BOTH SERVOS AT DEAD CENTER (90°)    ║");
    Serial.println("╚═══════════════════════════════════════╝\n");
    
    Serial.println("INSTRUCTIONS:");
    Serial.println("-------------");
    Serial.println("1. Check if servo arms are perpendicular to body");
    Serial.println("2. If not centered, adjust servo horn position");
    Serial.println("3. Send commands via Serial Monitor:\n");
    
    Serial.println("COMMANDS:");
    Serial.println("---------");
    Serial.println("  0-180  -> Move to angle (e.g., '90')");
    Serial.println("  'r'    -> Control RIGHT servo only");
    Serial.println("  'l'    -> Control LEFT servo only");
    Serial.println("  'b'    -> Control BOTH servos");
    Serial.println("  'c'    -> Return to CENTER (90°)");
    Serial.println("  's'    -> SWEEP test (30° to 150°)");
    Serial.println("  'i'    -> Show current INFO\n");
    
    printStatus();
}

void loop() {
    // Check for serial input
    if (Serial.available() > 0) {
        String input = Serial.readStringUntil('\n');
        input.trim();
        
        if (input.length() > 0) {
            processCommand(input);
        }
    }
    
    delay(10);
}

void processCommand(String cmd) {
    cmd.toLowerCase();
    
    // Check for mode commands
    if (cmd == "r") {
        servoMode = RIGHT_ONLY;
        Serial.println("\n→ Mode: RIGHT servo only");
        printStatus();
        return;
    }
    else if (cmd == "l") {
        servoMode = LEFT_ONLY;
        Serial.println("\n→ Mode: LEFT servo only");
        printStatus();
        return;
    }
    else if (cmd == "b") {
        servoMode = BOTH;
        Serial.println("\n→ Mode: BOTH servos");
        printStatus();
        return;
    }
    else if (cmd == "c") {
        moveToAngle(CENTER);
        Serial.println("\n✓ Returned to CENTER (90°)");
        printStatus();
        return;
    }
    else if (cmd == "s") {
        sweepTest();
        return;
    }
    else if (cmd == "i") {
        printStatus();
        return;
    }
    
    // Try to parse as angle
    int angle = cmd.toInt();
    if (angle >= 0 && angle <= 180) {
        moveToAngle(angle);
        Serial.printf("\n✓ Moved to %d°\n", angle);
        printStatus();
    }
    else {
        Serial.println("\n✗ Invalid command! Try: 0-180, r, l, b, c, s, or i");
    }
}

void moveToAngle(int angle) {
    currentAngle = constrain(angle, 0, 180);
    
    switch(servoMode) {
        case BOTH:
            right_elevon.write(currentAngle);
            left_elevon.write(currentAngle);
            break;
        case RIGHT_ONLY:
            right_elevon.write(currentAngle);
            break;
        case LEFT_ONLY:
            left_elevon.write(currentAngle);
            break;
    }
}

void sweepTest() {
    Serial.println("\n→ Starting SWEEP test (30° → 150° → 30°)...");
    Serial.println("  Watch for smooth motion and no binding\n");
    
    // Sweep up
    for (int angle = 30; angle <= 150; angle += 5) {
        moveToAngle(angle);
        Serial.printf("  Angle: %d°\r", angle);
        delay(100);
    }
    
    Serial.println("\n  → Reversing...");
    delay(500);
    
    // Sweep down
    for (int angle = 150; angle >= 30; angle -= 5) {
        moveToAngle(angle);
        Serial.printf("  Angle: %d°\r", angle);
        delay(100);
    }
    
    // Return to center
    Serial.println("\n  → Returning to center...");
    delay(500);
    moveToAngle(CENTER);
    
    Serial.println("\n✓ Sweep test complete!");
    printStatus();
}

void printStatus() {
    Serial.println("\n┌───────────────────────────────────────┐");
    Serial.printf("│ Mode: %-31s │\n", 
        servoMode == BOTH ? "BOTH SERVOS" : 
        servoMode == RIGHT_ONLY ? "RIGHT SERVO ONLY" : 
        "LEFT SERVO ONLY");
    Serial.printf("│ Current Angle: %-18d° │\n", currentAngle);
    Serial.println("└───────────────────────────────────────┘");
    Serial.println("Ready for next command...\n");
}

