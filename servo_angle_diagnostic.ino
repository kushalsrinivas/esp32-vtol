/* SERVO ANGLE DIAGNOSTIC TOOL
   
   This code helps diagnose servo angle discrepancies.
   It will move servos through specific angles and show you what's actually happening.
   
   PROBLEM:
   ========
   - Code commands 90° but servo physically moves to ~45°
   - Need to find the TRUE angle values for your servos
   
   HARDWARE SETUP:
   ===============
   RIGHT ELEVON:
     Signal -> GPIO 25
     VCC    -> 5V (external power supply)
     GND    -> GND
   
   LEFT ELEVON:
     Signal -> GPIO 26
     VCC    -> 5V (external power supply)
     GND    -> GND
   
   INSTRUCTIONS:
   =============
   1. Upload this code
   2. Open Serial Monitor (115200 baud)
   3. The code will automatically test angles: 0°, 45°, 90°, 135°, 180°
   4. Watch the servo horn and note where it ACTUALLY points
   5. Use Serial commands to fine-tune and find true center
   
   WHAT TO LOOK FOR:
   =================
   - At commanded "90°", where does the horn actually point?
   - Does it point at 45°, 60°, 90°, or somewhere else?
   - This will tell us if we need to remap/calibrate angles
*/

#include <ESP32Servo.h>

// Pin definitions
#define RIGHT_ELEVON_PIN 25
#define LEFT_ELEVON_PIN 26

// Servo objects
Servo right_elevon;
Servo left_elevon;

// Test angles
const int testAngles[] = {0, 45, 90, 135, 180};
const int numAngles = 5;

void setup() {
    Serial.begin(115200);
    delay(2000);
    
    Serial.println("\n╔═══════════════════════════════════════╗");
    Serial.println("║   SERVO ANGLE DIAGNOSTIC TOOL        ║");
    Serial.println("╚═══════════════════════════════════════╝\n");
    
    Serial.println("This tool will help identify servo angle issues.\n");
    
    // Attach servos with explicit PWM range (some servos need this)
    Serial.println("→ Attaching servos with standard PWM range...");
    Serial.println("  Min PWM: 500µs | Max PWM: 2500µs\n");
    
    right_elevon.attach(RIGHT_ELEVON_PIN, 500, 2500);  // Standard servo range
    left_elevon.attach(LEFT_ELEVON_PIN, 500, 2500);
    delay(500);
    
    Serial.println("╔═══════════════════════════════════════╗");
    Serial.println("║  AUTOMATIC ANGLE TEST STARTING        ║");
    Serial.println("╚═══════════════════════════════════════╝\n");
    
    Serial.println("Watch the servo horns and note where they point!\n");
    
    // Test each angle
    for (int i = 0; i < numAngles; i++) {
        int angle = testAngles[i];
        
        Serial.println("┌───────────────────────────────────────┐");
        Serial.printf("│ COMMANDED ANGLE: %3d°                │\n", angle);
        Serial.println("└───────────────────────────────────────┘");
        Serial.println("  Moving both servos...");
        
        right_elevon.write(angle);
        left_elevon.write(angle);
        delay(1000);
        
        Serial.println("  → Servo horns should be at the SAME position");
        Serial.print("  → Where do they ACTUALLY point? ");
        if (angle == 0) {
            Serial.println("(full left/down)");
        } else if (angle == 90) {
            Serial.println("(CENTER - perpendicular!)");
        } else if (angle == 180) {
            Serial.println("(full right/up)");
        } else {
            Serial.printf("(%d° from reference)\n", angle);
        }
        Serial.println();
        
        delay(2000);
    }
    
    // Return to center
    Serial.println("╔═══════════════════════════════════════╗");
    Serial.println("║  TEST COMPLETE - RETURNING TO 90°    ║");
    Serial.println("╚═══════════════════════════════════════╝\n");
    
    right_elevon.write(90);
    left_elevon.write(90);
    delay(1000);
    
    // Print analysis
    Serial.println("╔═══════════════════════════════════════╗");
    Serial.println("║           ANALYSIS GUIDE              ║");
    Serial.println("╚═══════════════════════════════════════╝\n");
    
    Serial.println("RESULTS:");
    Serial.println("--------");
    Serial.println("1. If commanded 90° shows horn at 90° (perpendicular):");
    Serial.println("   ✓ Your servos are CORRECT! No changes needed.\n");
    
    Serial.println("2. If commanded 90° shows horn at 45°:");
    Serial.println("   → Your servo needs ANGLE REMAPPING");
    Serial.println("   → Try commanding 135° to get true 90°\n");
    
    Serial.println("3. If commanded 90° shows horn at 135°:");
    Serial.println("   → Your servo needs REVERSE REMAPPING");
    Serial.println("   → Try commanding 45° to get true 90°\n");
    
    Serial.println("4. If angles are completely off:");
    Serial.println("   → Your servo may need PWM calibration");
    Serial.println("   → Try different PWM ranges (1000-2000µs)\n");
    
    Serial.println("╔═══════════════════════════════════════╗");
    Serial.println("║      MANUAL TEST MODE ACTIVE          ║");
    Serial.println("╚═══════════════════════════════════════╝\n");
    
    Serial.println("COMMANDS:");
    Serial.println("---------");
    Serial.println("  Type angle (0-180) to test: '90'");
    Serial.println("  Type 'r' + angle for RIGHT only: 'r90'");
    Serial.println("  Type 'l' + angle for LEFT only: 'l90'");
    Serial.println("  Type 'pwm' to test different PWM ranges");
    Serial.println("  Type 'sweep' for full sweep test\n");
    
    Serial.println("Ready for manual commands...\n");
}

void loop() {
    if (Serial.available() > 0) {
        String input = Serial.readStringUntil('\n');
        input.trim();
        input.toLowerCase();
        
        if (input.length() > 0) {
            processCommand(input);
        }
    }
    
    delay(10);
}

void processCommand(String cmd) {
    if (cmd == "sweep") {
        sweepTest();
        return;
    }
    
    if (cmd == "pwm") {
        pwmTest();
        return;
    }
    
    // Check for r/l prefix
    bool rightOnly = false;
    bool leftOnly = false;
    
    if (cmd.startsWith("r")) {
        rightOnly = true;
        cmd = cmd.substring(1);
    } else if (cmd.startsWith("l")) {
        leftOnly = true;
        cmd = cmd.substring(1);
    }
    
    // Parse angle
    int angle = cmd.toInt();
    
    if (angle >= 0 && angle <= 180) {
        Serial.printf("\n→ Commanding %d°", angle);
        
        if (rightOnly) {
            Serial.println(" (RIGHT servo only)");
            right_elevon.write(angle);
        } else if (leftOnly) {
            Serial.println(" (LEFT servo only)");
            left_elevon.write(angle);
        } else {
            Serial.println(" (BOTH servos)");
            right_elevon.write(angle);
            left_elevon.write(angle);
        }
        
        Serial.println("  Where does the horn actually point?\n");
    } else {
        Serial.println("\n✗ Invalid command!");
    }
}

void sweepTest() {
    Serial.println("\n╔═══════════════════════════════════════╗");
    Serial.println("║         SWEEP TEST (0° → 180°)       ║");
    Serial.println("╚═══════════════════════════════════════╝\n");
    
    Serial.println("→ Watch for smooth motion and consistent speed\n");
    
    // Forward sweep
    for (int angle = 0; angle <= 180; angle += 10) {
        Serial.printf("  Angle: %3d°\r", angle);
        right_elevon.write(angle);
        left_elevon.write(angle);
        delay(200);
    }
    
    Serial.println("\n\n→ Reversing (180° → 0°)\n");
    delay(500);
    
    // Reverse sweep
    for (int angle = 180; angle >= 0; angle -= 10) {
        Serial.printf("  Angle: %3d°\r", angle);
        right_elevon.write(angle);
        left_elevon.write(angle);
        delay(200);
    }
    
    // Return to center
    Serial.println("\n\n→ Returning to 90°\n");
    right_elevon.write(90);
    left_elevon.write(90);
    
    Serial.println("✓ Sweep complete!\n");
}

void pwmTest() {
    Serial.println("\n╔═══════════════════════════════════════╗");
    Serial.println("║       PWM CALIBRATION TEST            ║");
    Serial.println("╚═══════════════════════════════════════╝\n");
    
    Serial.println("Testing different PWM ranges...\n");
    
    // Test different PWM ranges
    int pwmRanges[][2] = {
        {500, 2500},   // Standard
        {1000, 2000},  // Common alternative
        {600, 2400},   // Slightly narrower
        {544, 2400}    // Arduino Servo library default
    };
    
    for (int i = 0; i < 4; i++) {
        int minPWM = pwmRanges[i][0];
        int maxPWM = pwmRanges[i][1];
        
        Serial.printf("→ Testing PWM range: %d-%d µs\n", minPWM, maxPWM);
        
        // Reattach with new range
        right_elevon.detach();
        left_elevon.detach();
        delay(100);
        
        right_elevon.attach(RIGHT_ELEVON_PIN, minPWM, maxPWM);
        left_elevon.attach(LEFT_ELEVON_PIN, minPWM, maxPWM);
        delay(500);
        
        // Test 90°
        Serial.println("  Setting to 90°...");
        right_elevon.write(90);
        left_elevon.write(90);
        delay(2000);
        
        Serial.println("  → Check if horn is at 90° (perpendicular)\n");
        delay(1000);
    }
    
    // Restore standard range
    Serial.println("→ Restoring standard PWM range (500-2500µs)\n");
    right_elevon.detach();
    left_elevon.detach();
    delay(100);
    right_elevon.attach(RIGHT_ELEVON_PIN, 500, 2500);
    left_elevon.attach(LEFT_ELEVON_PIN, 500, 2500);
    right_elevon.write(90);
    left_elevon.write(90);
    
    Serial.println("✓ PWM test complete!\n");
}

