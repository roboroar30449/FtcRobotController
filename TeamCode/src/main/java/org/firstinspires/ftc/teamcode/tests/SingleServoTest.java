package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Joystick Servo Test", group = "Test")
public class SingleServoTest extends OpMode {

    private Servo testServo;

    private static final double MAX_SERVO_ROTATION = 300.0; // degrees
    private static final double MIN_POS = 0.0;
    private static final double MAX_POS = 1.0;

    private double servoPosition = 0.2; // Start at middle position

    @Override
    public void init() {
        testServo = hardwareMap.get(Servo.class, "ClawOC"); // Change if testing a different servo
        testServo.setPosition(servoPosition);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Joystick Y is inverted (up is negative), so invert it
        double joystickInput = -gamepad1.left_stick_y;

        // Adjust sensitivity (optional)
        double sensitivity = 0.005;

        // Change position based on joystick input
        servoPosition += joystickInput * sensitivity;

        // Clamp position to valid servo range
        servoPosition = Math.max(MIN_POS, Math.min(MAX_POS, servoPosition));

        // Apply position to servo
        testServo.setPosition(servoPosition);

        // Output current position in degrees
        double currentDegrees = servoPosition * MAX_SERVO_ROTATION;
        telemetry.addData("Servo Position (0.0 - 1.0):", servoPosition);
        telemetry.addData("Servo Position (degrees):", currentDegrees);
        telemetry.update();
    }
}
