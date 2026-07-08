/**
 * @file main.cpp
 * @brief 众灵科技 (Zhongling) ZP Series Bus Servo — Interactive Test Program
 *
 * Serial (USB CDC) menu-driven test for Zhongling ZP series bus servos.
 *
 * Hardware:
 *  - ESP32S3 TX (GPIO 17) → Zhongling driver board RX
 *  - ESP32S3 RX (GPIO 18) ← Zhongling driver board TX (for response reading)
 *  - Driver board → Bus servo (handles half-duplex conversion internally)
 *  - Servo VCC → external power (typically 6~8.4V)
 *  - Common GND between ESP32, driver board, and servo power
 *
 * Baud rates:
 *  - Servo bus:  115200 (default, matches factory setting)
 *  - Debug serial: 115200 (USB CDC)
 */

#include <Arduino.h>
#include "ZServoDriver.h"

// ──────────────────────────────────────────────
//  Pin / Port Configuration
// ──────────────────────────────────────────────
#define SERVO_SERIAL   Serial1
#define SERVO_BAUD     115200
#define SERVO_DIR_PIN  -1        // Direction pin for RS-485 (-1 = not used)
#define DEBUG_SERIAL   Serial
#define DEBUG_BAUD     115200

// ──────────────────────────────────────────────
//  Servo Driver Instance
// ──────────────────────────────────────────────
ZServoDriver servo;

// ──────────────────────────────────────────────
//  Utility: Read integer from serial
// ──────────────────────────────────────────────

static int readInt(const char *prompt, int defaultVal = 0) {
    DEBUG_SERIAL.print(prompt);
    if (defaultVal != 0) {
        DEBUG_SERIAL.print(" [");
        DEBUG_SERIAL.print(defaultVal);
        DEBUG_SERIAL.print("]");
    }
    DEBUG_SERIAL.print(": ");

    unsigned long start = millis();
    while (millis() - start < 15000) {
        if (DEBUG_SERIAL.available()) {
            String input = DEBUG_SERIAL.readStringUntil('\n');
            input.trim();
            if (input.length() == 0) return defaultVal;
            return input.toInt();
        }
        delay(10);
    }
    DEBUG_SERIAL.println(defaultVal);
    return defaultVal;
}

static void waitEnter() {
    DEBUG_SERIAL.println("\nPress Enter to continue...");
    while (true) {
        if (DEBUG_SERIAL.available()) {
            while (DEBUG_SERIAL.available()) DEBUG_SERIAL.read();
            break;
        }
        delay(10);
    }
}

// ──────────────────────────────────────────────
//  Test 1: ID Scan
// ──────────────────────────────────────────────

static void testIDScan() {
    DEBUG_SERIAL.println("\n========== ID Scan ==========");
    DEBUG_SERIAL.println("Scans for servos by requesting their ID.");
    DEBUG_SERIAL.println("Note: Only works reliably with one servo on the bus.");
    DEBUG_SERIAL.println("      For multiple servos, scan each ID individually.\n");

    int startId = readInt("Start ID", 0);
    int endId   = readInt("End ID", 20);
    if (endId < startId) { int t = startId; startId = endId; endId = t; }
    if (endId > 254) endId = 254;

    DEBUG_SERIAL.printf("\nScanning IDs %d ~ %d ...\n", startId, endId);
    DEBUG_SERIAL.println("(Send readAngle to each ID — responding servos will reply)\n");

    int found = 0;
    char rspBuf[ZSERVO_RSP_BUF_SIZE];

    for (int id = startId; id <= endId; id++) {
        servo.readAngle((uint8_t)id);
        delay(15);  // Wait for possible response

        int rspLen = servo.readResponse(rspBuf, sizeof(rspBuf), 30);
        if (rspLen > 0) {
            uint8_t rspId;
            uint16_t rspPwm;
            if (ZServoDriver::parseAngleResponse(rspBuf, &rspId, &rspPwm)) {
                DEBUG_SERIAL.printf("  [FOUND] ID=%d  PWM=%d\n", rspId, rspPwm);
                found++;
            } else {
                DEBUG_SERIAL.printf("  [?]     Raw response: %s\n", rspBuf);
            }
        } else {
            DEBUG_SERIAL.printf("  [----]  ID=%d  no response\n", id);
        }
    }

    DEBUG_SERIAL.printf("\nScan complete: %d servo(s) responded.\n", found);
    waitEnter();
}

// ──────────────────────────────────────────────
//  Test 2: Single Servo Position Control
// ──────────────────────────────────────────────

static void testPositionControl() {
    DEBUG_SERIAL.println("\n========== Position Control ==========");

    int id = readInt("Servo ID", 1);
    if (id < 0 || id > 254) {
        DEBUG_SERIAL.println("Invalid ID (0~254).");
        return;
    }

    DEBUG_SERIAL.printf("\nServo %d — Enter PWM positions (500~2500, 1500=mid)\n", id);
    DEBUG_SERIAL.println("Enter -1 to exit.\n");

    while (true) {
        int pwm = readInt("PWM", 1500);
        if (pwm < 0) break;

        // Clamp to valid range
        if (pwm < ZSERVO_PWM_MIN) pwm = ZSERVO_PWM_MIN;
        if (pwm > ZSERVO_PWM_MAX) pwm = ZSERVO_PWM_MAX;

        int timeMs = readInt("Time (ms)", 1000);
        if (timeMs < 0) timeMs = 0;
        if (timeMs > ZSERVO_TIME_MAX) timeMs = ZSERVO_TIME_MAX;

        DEBUG_SERIAL.printf("Moving servo %d to PWM=%d in %d ms... ", id, pwm, timeMs);
        if (servo.moveServo((uint8_t)id, (uint16_t)pwm, (uint16_t)timeMs)) {
            DEBUG_SERIAL.println("Sent!");

            // Read back angle after movement
            if (timeMs > 0) {
                delay(timeMs + 100);  // Wait for move to complete + margin
            } else {
                delay(300);
            }

            servo.readAngle((uint8_t)id);
            char rspBuf[ZSERVO_RSP_BUF_SIZE];
            int rspLen = servo.readResponse(rspBuf, sizeof(rspBuf), 100);
            if (rspLen > 0) {
                uint8_t rspId;
                uint16_t rspPwm;
                if (ZServoDriver::parseAngleResponse(rspBuf, &rspId, &rspPwm)) {
                    DEBUG_SERIAL.printf("  → Current: ID=%d PWM=%d\n", rspId, rspPwm);
                } else {
                    DEBUG_SERIAL.printf("  → Raw response: %s\n", rspBuf);
                }
            }
        } else {
            DEBUG_SERIAL.println("Failed to send!");
        }
    }
}

// ──────────────────────────────────────────────
//  Test 3: Multi-Servo Sync Control
// ──────────────────────────────────────────────

static void testMultiServo() {
    DEBUG_SERIAL.println("\n========== Multi-Servo Sync Control ==========");
    DEBUG_SERIAL.println("All servos in the group move simultaneously.\n");

    int count = readInt("Number of servos", 2);
    if (count < 1 || count > 16) {
        DEBUG_SERIAL.println("Count must be 1~16.");
        return;
    }

    ServoPose poses[16];

    for (int i = 0; i < count; i++) {
        DEBUG_SERIAL.printf("\n--- Servo %d/%d ---\n", i + 1, count);
        poses[i].id     = (uint8_t)readInt("  ID", i + 1);
        int pwm         = readInt("  PWM (500~2500)", 1500);
        poses[i].timeMs = (uint16_t)readInt("  Time (ms)", 1000);

        if (pwm < ZSERVO_PWM_MIN) pwm = ZSERVO_PWM_MIN;
        if (pwm > ZSERVO_PWM_MAX) pwm = ZSERVO_PWM_MAX;
        poses[i].pwm = (uint16_t)pwm;
    }

    DEBUG_SERIAL.println("\nSending multi-servo command...");
    if (servo.moveServos(poses, (uint8_t)count)) {
        DEBUG_SERIAL.printf("Sent: %s\n", servo.getLastCommand());
    } else {
        DEBUG_SERIAL.println("Failed to send!");
    }

    waitEnter();
}

// ──────────────────────────────────────────────
//  Test 4: Read Angle Feedback (continuous)
// ──────────────────────────────────────────────

static void testReadAngle() {
    DEBUG_SERIAL.println("\n========== Read Angle Feedback ==========");

    int id = readInt("Servo ID", 1);
    if (id < 0 || id > 254) {
        DEBUG_SERIAL.println("Invalid ID.");
        return;
    }

    DEBUG_SERIAL.println("\nReading angle continuously. Press any key to stop...\n");
    DEBUG_SERIAL.println("ID  |  PWM   | Raw Response");
    DEBUG_SERIAL.println("----|--------|-------------");

    char rspBuf[ZSERVO_RSP_BUF_SIZE];
    unsigned long lastRead = 0;

    while (!DEBUG_SERIAL.available()) {
        if (millis() - lastRead < 300) {
            delay(10);
            continue;
        }
        lastRead = millis();

        servo.readAngle((uint8_t)id);
        int rspLen = servo.readResponse(rspBuf, sizeof(rspBuf), 100);

        if (rspLen > 0) {
            uint8_t rspId;
            uint16_t rspPwm;
            if (ZServoDriver::parseAngleResponse(rspBuf, &rspId, &rspPwm)) {
                DEBUG_SERIAL.printf(" %d  | %5d  | %s\n", rspId, rspPwm, rspBuf);
            } else {
                DEBUG_SERIAL.printf(" ?  |   ?    | %s\n", rspBuf);
            }
        } else {
            DEBUG_SERIAL.println("--- |  ----  | (timeout)");
        }
    }

    // Clear input buffer
    while (DEBUG_SERIAL.available()) DEBUG_SERIAL.read();
    DEBUG_SERIAL.println("\nStopped.");
}

// ──────────────────────────────────────────────
//  Test 5: Mode Control
// ──────────────────────────────────────────────

static void testModeControl() {
    DEBUG_SERIAL.println("\n========== Mode Control ==========");

    int id = readInt("Servo ID", 1);
    if (id < 0 || id > 254) {
        DEBUG_SERIAL.println("Invalid ID.");
        return;
    }

    while (true) {
        DEBUG_SERIAL.println("\n--- Mode Menu ---");
        DEBUG_SERIAL.println("  1. Read current mode");
        DEBUG_SERIAL.println("  2. 270° CW  (mode 1)");
        DEBUG_SERIAL.println("  3. 270° CCW (mode 2)");
        DEBUG_SERIAL.println("  4. 180° CW  (mode 3)");
        DEBUG_SERIAL.println("  5. 180° CCW (mode 4)");
        DEBUG_SERIAL.println("  6. Motor 360° CW  (mode 5)");
        DEBUG_SERIAL.println("  7. Motor 360° CCW (mode 6)");
        DEBUG_SERIAL.println("  8. Motor 360° CW alt  (mode 7)");
        DEBUG_SERIAL.println("  9. Motor 360° CCW alt (mode 8)");
        DEBUG_SERIAL.println("  0. Exit");

        int choice = readInt("Choice", 0);
        if (choice == 0) break;

        if (choice == 1) {
            DEBUG_SERIAL.print("Reading mode... ");
            servo.readMode((uint8_t)id);
            char rspBuf[ZSERVO_RSP_BUF_SIZE];
            int rspLen = servo.readResponse(rspBuf, sizeof(rspBuf), 200);
            if (rspLen > 0) {
                DEBUG_SERIAL.printf("Response: %s\n", rspBuf);
            } else {
                DEBUG_SERIAL.println("No response.");
            }
        } else if (choice >= 2 && choice <= 9) {
            uint8_t mode = (uint8_t)(choice - 1);  // Map menu 2-9 → mode 1-8
            DEBUG_SERIAL.printf("Setting mode %d... ", mode);
            if (servo.setMode((uint8_t)id, mode)) {
                DEBUG_SERIAL.println("Sent!");
            } else {
                DEBUG_SERIAL.println("Failed!");
            }
        } else {
            DEBUG_SERIAL.println("Invalid choice.");
        }
    }
}

// ──────────────────────────────────────────────
//  Test 6: Configuration
// ──────────────────────────────────────────────

static void testConfiguration() {
    DEBUG_SERIAL.println("\n========== Configuration ==========");

    int id = readInt("Servo ID", 1);
    if (id < 0 || id > 254) {
        DEBUG_SERIAL.println("Invalid ID.");
        return;
    }

    while (true) {
        DEBUG_SERIAL.println("\n--- Config Menu ---");
        DEBUG_SERIAL.println("  1. Read servo ID");
        DEBUG_SERIAL.println("  2. Change servo ID");
        DEBUG_SERIAL.println("  3. Read firmware version");
        DEBUG_SERIAL.println("  4. Change baud rate");
        DEBUG_SERIAL.println("  5. Factory reset");
        DEBUG_SERIAL.println("  6. Set current position as boot default");
        DEBUG_SERIAL.println("  7. Set boot torque-off mode");
        DEBUG_SERIAL.println("  8. Calibrate mid-point offset");
        DEBUG_SERIAL.println("  9. Set PID parameters");
        DEBUG_SERIAL.println(" 10. Set protection temperature");
        DEBUG_SERIAL.println(" 11. LED ON");
        DEBUG_SERIAL.println(" 12. LED OFF");
        DEBUG_SERIAL.println(" 13. Read voltage & temperature");
        DEBUG_SERIAL.println("  0. Exit");

        int choice = readInt("Choice", 0);
        if (choice == 0) break;

        char rspBuf[ZSERVO_RSP_BUF_SIZE];

        switch (choice) {
        case 1:  // Read ID
            DEBUG_SERIAL.print("Reading ID... ");
            servo.readID(id);
            if (servo.readResponse(rspBuf, sizeof(rspBuf), 200) > 0) {
                DEBUG_SERIAL.printf("Response: %s\n", rspBuf);
            } else {
                DEBUG_SERIAL.println("No response.");
            }
            break;

        case 2: {  // Change ID
            int newId = readInt("New ID (0~254)", 1);
            if (newId < 0 || newId > 254) {
                DEBUG_SERIAL.println("Invalid ID.");
                break;
            }
            DEBUG_SERIAL.printf("Changing ID %d → %d... ", id, newId);
            if (servo.setID((uint8_t)id, (uint8_t)newId)) {
                DEBUG_SERIAL.println("Sent! Remember to use new ID for subsequent commands.");
                id = newId;  // Update local variable
            } else {
                DEBUG_SERIAL.println("Failed!");
            }
            break;
        }

        case 3:  // Read version
            DEBUG_SERIAL.print("Reading version... ");
            servo.readVersion((uint8_t)id);
            if (servo.readResponse(rspBuf, sizeof(rspBuf), 200) > 0) {
                DEBUG_SERIAL.printf("Response: %s\n", rspBuf);
            } else {
                DEBUG_SERIAL.println("No response.");
            }
            break;

        case 4: {  // Change baud rate
            DEBUG_SERIAL.println("Baud rate codes:");
            DEBUG_SERIAL.println("  1=9600  2=19200  3=38400  4=57600");
            DEBUG_SERIAL.println("  5=115200  6=128000  7=256000  8=1000000");
            int code = readInt("Baud code (1~8)", 5);
            if (code < 1 || code > 8) {
                DEBUG_SERIAL.println("Invalid code.");
                break;
            }
            DEBUG_SERIAL.printf("Setting baud rate code %d... ", code);
            if (servo.setBaud((uint8_t)id, (uint8_t)code)) {
                DEBUG_SERIAL.println("Sent!");
                DEBUG_SERIAL.println("IMPORTANT: Update SERVO_BAUD and re-upload to match!");
            } else {
                DEBUG_SERIAL.println("Failed!");
            }
            break;
        }

        case 5:  // Factory reset
            DEBUG_SERIAL.println("WARNING: This will reset ALL settings including ID!");
            if (readInt("Enter 1 to confirm", 0) == 1) {
                DEBUG_SERIAL.printf("Resetting servo %d... ", id);
                if (servo.factoryReset((uint8_t)id)) {
                    DEBUG_SERIAL.println("Sent!");
                } else {
                    DEBUG_SERIAL.println("Failed!");
                }
            } else {
                DEBUG_SERIAL.println("Cancelled.");
            }
            break;

        case 6:  // Set boot value
            DEBUG_SERIAL.printf("Setting current position as boot default for servo %d... ", id);
            DEBUG_SERIAL.println(servo.setBootValue((uint8_t)id) ? "Sent!" : "Failed!");
            break;

        case 7:  // Set boot torque-off
            DEBUG_SERIAL.printf("Setting boot torque-off for servo %d... ", id);
            DEBUG_SERIAL.println(servo.setBootTorqueOff((uint8_t)id) ? "Sent!" : "Failed!");
            break;

        case 8: {  // Mid-point calibration
            int offset = readInt("Offset from 1500 (e.g. +50 or -50)", 0);
            DEBUG_SERIAL.printf("Calibrating mid offset %+d... ", offset);
            DEBUG_SERIAL.println(servo.setMidOffset((uint8_t)id, (int16_t)offset) ? "Sent!" : "Failed!");
            break;
        }

        case 9: {  // Set PID
            int kp = readInt("KP (0~999)", 100);
            int ki = readInt("KI (0~999)", 10);
            if (kp < 0 || kp > 999 || ki < 0 || ki > 999) {
                DEBUG_SERIAL.println("Values out of range.");
                break;
            }
            DEBUG_SERIAL.printf("Setting KP=%d KI=%d... ", kp, ki);
            DEBUG_SERIAL.println(servo.setPID((uint8_t)id, (uint16_t)kp, (uint16_t)ki) ? "Sent!" : "Failed!");
            break;
        }

        case 10: {  // Protection temp
            int temp = readInt("Temperature threshold °C (25~80)", 60);
            if (temp < 25 || temp > 80) {
                DEBUG_SERIAL.println("Value out of range.");
                break;
            }
            DEBUG_SERIAL.printf("Setting protection temp %d°C... ", temp);
            DEBUG_SERIAL.println(servo.setProtectionTemp((uint8_t)id, (uint8_t)temp) ? "Sent!" : "Failed!");
            break;
        }

        case 11:  // LED ON
            DEBUG_SERIAL.printf("Turning LED ON for servo %d... ", id);
            DEBUG_SERIAL.println(servo.ledOn((uint8_t)id) ? "Sent!" : "Failed!");
            break;

        case 12:  // LED OFF
            DEBUG_SERIAL.printf("Turning LED OFF for servo %d... ", id);
            DEBUG_SERIAL.println(servo.ledOff((uint8_t)id) ? "Sent!" : "Failed!");
            break;

        case 13:  // Read voltage & temp
            DEBUG_SERIAL.print("Reading voltage & temperature... ");
            servo.readVoltageTemp((uint8_t)id);
            if (servo.readResponse(rspBuf, sizeof(rspBuf), 200) > 0) {
                DEBUG_SERIAL.printf("Response: %s\n", rspBuf);
            } else {
                DEBUG_SERIAL.println("No response.");
            }
            break;

        default:
            DEBUG_SERIAL.println("Invalid choice.");
            break;
        }
    }
}

// ──────────────────────────────────────────────
//  Test 7: Automatic Sweep Test
// ──────────────────────────────────────────────

static void testSweep() {
    DEBUG_SERIAL.println("\n========== Automatic Sweep Test ==========");

    int id = readInt("Servo ID", 1);
    if (id < 0 || id > 254) {
        DEBUG_SERIAL.println("Invalid ID.");
        return;
    }

    int timeMs  = readInt("Time per sweep step (ms)", 1000);
    int stepSize = readInt("PWM step size", 100);
    int cycles  = readInt("Number of cycles (0=infinite)", 3);

    if (timeMs < 100) timeMs = 100;
    if (stepSize < 10) stepSize = 10;
    if (stepSize > 500) stepSize = 500;

    DEBUG_SERIAL.println("\nStarting sweep. Press any key to stop.");
    DEBUG_SERIAL.printf("Range: %d → %d → %d, step=%d, time=%d ms\n",
                        ZSERVO_PWM_MIN, ZSERVO_PWM_MAX, ZSERVO_PWM_MIN, stepSize, timeMs);

    int cycle = 0;
    while ((cycles == 0) || (cycle < cycles)) {
        if (DEBUG_SERIAL.available()) {
            while (DEBUG_SERIAL.available()) DEBUG_SERIAL.read();
            break;
        }

        cycle++;
        DEBUG_SERIAL.printf("\n--- Cycle %d ---\n", cycle);

        // Move to max
        DEBUG_SERIAL.printf("  Moving to %d... ", ZSERVO_PWM_MAX);
        servo.moveServo((uint8_t)id, ZSERVO_PWM_MAX, (uint16_t)timeMs);
        delay(timeMs + 200);

        if (DEBUG_SERIAL.available()) break;

        // Read position
        servo.readAngle((uint8_t)id);
        char rspBuf[ZSERVO_RSP_BUF_SIZE];
        if (servo.readResponse(rspBuf, sizeof(rspBuf), 100) > 0) {
            uint8_t rspId; uint16_t rspPwm;
            if (ZServoDriver::parseAngleResponse(rspBuf, &rspId, &rspPwm)) {
                DEBUG_SERIAL.printf("Actual PWM=%d\n", rspPwm);
            }
        }

        if (DEBUG_SERIAL.available()) break;

        // Move to min
        DEBUG_SERIAL.printf("  Moving to %d... ", ZSERVO_PWM_MIN);
        servo.moveServo((uint8_t)id, ZSERVO_PWM_MIN, (uint16_t)timeMs);
        delay(timeMs + 200);

        if (DEBUG_SERIAL.available()) break;

        // Move back to mid
        DEBUG_SERIAL.printf("  Moving to %d (mid)... ", ZSERVO_PWM_MID);
        servo.moveServo((uint8_t)id, ZSERVO_PWM_MID, (uint16_t)timeMs);
        delay(timeMs + 200);

        DEBUG_SERIAL.println("done.");
    }

    // Return to mid
    servo.moveServo((uint8_t)id, ZSERVO_PWM_MID, 500);
    DEBUG_SERIAL.println("\nSweep stopped. Servo returned to mid position.");
}

// ──────────────────────────────────────────────
//  Test 8: Torque Control
// ──────────────────────────────────────────────

static void testTorque() {
    DEBUG_SERIAL.println("\n========== Torque Control ==========");

    int id = readInt("Servo ID", 1);
    if (id < 0 || id > 254) {
        DEBUG_SERIAL.println("Invalid ID.");
        return;
    }

    while (true) {
        DEBUG_SERIAL.println("\n--- Torque Menu ---");
        DEBUG_SERIAL.println("  1. Torque ON  (hold position)");
        DEBUG_SERIAL.println("  2. Torque OFF (free spinning)");
        DEBUG_SERIAL.println("  3. Torque OFF (no resistance — PULM)");
        DEBUG_SERIAL.println("  4. Stop servo (PDST)");
        DEBUG_SERIAL.println("  5. Pause servo (PDPT)");
        DEBUG_SERIAL.println("  6. Resume servo (PDCT)");
        DEBUG_SERIAL.println("  0. Exit");

        int choice = readInt("Choice", 0);
        if (choice == 0) break;

        switch (choice) {
        case 1:
            DEBUG_SERIAL.printf("Enabling torque on servo %d... ", id);
            DEBUG_SERIAL.println(servo.torqueOn((uint8_t)id) ? "Sent!" : "Failed!");
            break;
        case 2:
            DEBUG_SERIAL.printf("Releasing torque on servo %d... ", id);
            DEBUG_SERIAL.println(servo.torqueOff((uint8_t)id) ? "Sent!" : "Failed!");
            break;
        case 3:
            DEBUG_SERIAL.printf("Releasing torque (no resistance) on servo %d... ", id);
            DEBUG_SERIAL.println(servo.torqueOffNoResist((uint8_t)id) ? "Sent!" : "Failed!");
            break;
        case 4:
            DEBUG_SERIAL.printf("Stopping servo %d... ", id);
            DEBUG_SERIAL.println(servo.stopServo((uint8_t)id) ? "Sent!" : "Failed!");
            break;
        case 5:
            DEBUG_SERIAL.printf("Pausing servo %d... ", id);
            DEBUG_SERIAL.println(servo.pauseServo((uint8_t)id) ? "Sent!" : "Failed!");
            break;
        case 6:
            DEBUG_SERIAL.printf("Resuming servo %d... ", id);
            DEBUG_SERIAL.println(servo.resumeServo((uint8_t)id) ? "Sent!" : "Failed!");
            break;
        default:
            DEBUG_SERIAL.println("Invalid choice.");
            break;
        }
    }
}

// ──────────────────────────────────────────────
//  Test 9: Quick Test (automatic)
// ──────────────────────────────────────────────

static void testQuick() {
    DEBUG_SERIAL.println("\n========== Quick Test ==========\n");

    char rspBuf[ZSERVO_RSP_BUF_SIZE];

    // 1. Scan for servos (ID 0~5)
    DEBUG_SERIAL.println("[1/4] Scanning for servos (ID 0~5)...");
    int foundId = -1;
    for (int id = 0; id <= 5; id++) {
        DEBUG_SERIAL.printf("  Probing ID=%d ... ", id);
        servo.readAngle((uint8_t)id);

        if (servo.readResponse(rspBuf, sizeof(rspBuf), 50) > 0) {
            uint8_t rspId; uint16_t rspPwm;
            if (ZServoDriver::parseAngleResponse(rspBuf, &rspId, &rspPwm)) {
                DEBUG_SERIAL.printf("FOUND! (PWM=%d)\n", rspPwm);
                if (foundId < 0) foundId = rspId;
            } else {
                DEBUG_SERIAL.printf("Raw: %s\n", rspBuf);
            }
        } else {
            DEBUG_SERIAL.println("none");
        }
        delay(20);
    }

    if (foundId < 0) {
        DEBUG_SERIAL.println("\nNo servos found! Check wiring and power.");
        DEBUG_SERIAL.println("Quick test aborted.");
        waitEnter();
        return;
    }

    DEBUG_SERIAL.printf("\nUsing first found servo: ID=%d\n\n", foundId);

    // 2. Read current position
    DEBUG_SERIAL.println("[2/4] Reading current position...");
    servo.readAngle((uint8_t)foundId);
    uint16_t currentPwm = ZSERVO_PWM_MID;
    if (servo.readResponse(rspBuf, sizeof(rspBuf), 200) > 0) {
        uint8_t rspId; uint16_t rspPwm;
        if (ZServoDriver::parseAngleResponse(rspBuf, &rspId, &rspPwm)) {
            DEBUG_SERIAL.printf("  Current: ID=%d PWM=%d\n", rspId, rspPwm);
            currentPwm = rspPwm;
        } else {
            DEBUG_SERIAL.printf("  Raw: %s\n", rspBuf);
        }
    }
    delay(200);

    // 3. Move to different positions
    DEBUG_SERIAL.println("[3/4] Testing movement...");

    uint16_t testPositions[] = { ZSERVO_PWM_MIN, ZSERVO_PWM_MAX, ZSERVO_PWM_MID };
    const char* testNames[] = { "min (500)", "max (2500)", "mid (1500)" };

    for (int i = 0; i < 3; i++) {
        DEBUG_SERIAL.printf("  Moving to %s... ", testNames[i]);
        servo.moveServo((uint8_t)foundId, testPositions[i], 1000);
        delay(1200);

        servo.readAngle((uint8_t)foundId);
        if (servo.readResponse(rspBuf, sizeof(rspBuf), 100) > 0) {
            uint8_t rspId; uint16_t rspPwm;
            if (ZServoDriver::parseAngleResponse(rspBuf, &rspId, &rspPwm)) {
                DEBUG_SERIAL.printf("Actual: PWM=%d\n", rspPwm);
            } else {
                DEBUG_SERIAL.printf("Raw: %s\n", rspBuf);
            }
        }
        delay(300);
    }

    // 4. Read voltage and temperature
    DEBUG_SERIAL.println("[4/4] Reading voltage & temperature...");
    servo.readVoltageTemp((uint8_t)foundId);
    if (servo.readResponse(rspBuf, sizeof(rspBuf), 200) > 0) {
        DEBUG_SERIAL.printf("  Response: %s\n", rspBuf);
    } else {
        DEBUG_SERIAL.println("  No response.");
    }

    DEBUG_SERIAL.println("\nQuick test complete!");
    waitEnter();
}

// ──────────────────────────────────────────────
//  Main Menu
// ──────────────────────────────────────────────

static void printMenu() {
    DEBUG_SERIAL.println("\n\n");
    DEBUG_SERIAL.println("╔══════════════════════════════════════════╗");
    DEBUG_SERIAL.println("║  Zhongling ZP Bus Servo Test v2.0      ║");
    DEBUG_SERIAL.println("╠══════════════════════════════════════════╣");
    DEBUG_SERIAL.println("║  1. ID Scan (find servos on bus)       ║");
    DEBUG_SERIAL.println("║  2. Single Servo Position Control      ║");
    DEBUG_SERIAL.println("║  3. Multi-Servo Sync Control           ║");
    DEBUG_SERIAL.println("║  4. Read Angle Feedback (continuous)   ║");
    DEBUG_SERIAL.println("║  5. Mode Control                       ║");
    DEBUG_SERIAL.println("║  6. Configuration (ID/Baud/PID/etc.)   ║");
    DEBUG_SERIAL.println("║  7. Automatic Sweep Test               ║");
    DEBUG_SERIAL.println("║  8. Torque & Motion Control            ║");
    DEBUG_SERIAL.println("║  9. Quick Test (auto-detect + move)    ║");
    DEBUG_SERIAL.println("║  0. Exit (deep sleep)                  ║");
    DEBUG_SERIAL.println("╠══════════════════════════════════════════╣");
    DEBUG_SERIAL.println("║  HW: ESP32S3, TX=GPIO17, RX=GPIO18    ║");
    DEBUG_SERIAL.println("║  Bus: Serial1, 115200 bps (Zhongling) ║");
    DEBUG_SERIAL.println("║  Protocol: ASCII (#000P1500T1000!)    ║");
    DEBUG_SERIAL.println("╚══════════════════════════════════════════╝");
}

// ──────────────────────────────────────────────
//  Setup & Loop
// ──────────────────────────────────────────────

void setup() {
    // Debug serial (USB CDC)
    DEBUG_SERIAL.begin(DEBUG_BAUD);
    delay(500);

    DEBUG_SERIAL.println("\n\n=== Zhongling ZP Series Bus Servo Driver ===");
    DEBUG_SERIAL.println("Initializing...");

    // Initialize servo serial
    // TX=GPIO17 → driver board RX
    // RX=GPIO18 ← driver board TX
    SERVO_SERIAL.setPins(17, 18);
    servo.begin(SERVO_SERIAL, SERVO_BAUD, SERVO_DIR_PIN);

    DEBUG_SERIAL.println("Servo driver ready!");
    DEBUG_SERIAL.println("Protocol: ASCII text (#000P1500T1000!)");
    DEBUG_SERIAL.println("Wiring:  ESP32 TX(GPIO17) → Driver board RX");
    DEBUG_SERIAL.println("         ESP32 RX(GPIO18) ← Driver board TX");
    DEBUG_SERIAL.println("         Driver board → Bus servo");
    DEBUG_SERIAL.println("         Servo VCC → external power (6~8.4V)");
    DEBUG_SERIAL.println("         Common GND required!");
}

void loop() {
    printMenu();
    int choice = readInt("\nSelect test", 9);

    switch (choice) {
    case 1:  testIDScan();          break;
    case 2:  testPositionControl(); break;
    case 3:  testMultiServo();      break;
    case 4:  testReadAngle();       break;
    case 5:  testModeControl();     break;
    case 6:  testConfiguration();   break;
    case 7:  testSweep();           break;
    case 8:  testTorque();          break;
    case 9:  testQuick();           break;
    case 0:
        DEBUG_SERIAL.println("\nEntering deep sleep. Reset to wake.");
        delay(100);
        esp_deep_sleep_start();
        break;
    default:
        DEBUG_SERIAL.println("Invalid choice.");
        break;
    }
}
