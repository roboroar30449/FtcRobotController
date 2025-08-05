package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.robot.MechController;

@TeleOp(name = "Servo Motor Test", group = "Testing")
public class ServoTest extends OpMode {

    private Servo clawOC, clawRot, headRot;

    // Constants for servo offsets and max rotation
    private static final double CLAW_OFFSET = 30; // Degrees
    private static final double CLAW_ROT_OFFSET = 150; // Degrees
    private static final double HEAD_ROT_OFFSET = 30; // Degrees
    private static final double MAX_SERVO_ROTATION = 300.0;
    MechController mechController;

    @Override
    public void init() {
        clawOC = hardwareMap.get(Servo.class, "ClawOC");
        clawRot = hardwareMap.get(Servo.class, "ClawRot");
        headRot = hardwareMap.get(Servo.class, "HeadRot");
    }

    @Override
    public void loop() {
        if (gamepad1.a) {
            setServoPositions(0, 0, 0);
        }
        if (gamepad1.x) {
            setServoPositions(0, -90, 0);
        }
        if (gamepad1.y) {
            setServoPositions(60, 90, 115);
        }
        telemetry.addData("Claw OC", mechController.ClawOCState());
        telemetry.addData("Claw Pos: ", mechController.ClawRotState());
        telemetry.addData("Head Pos: ", mechController.HeadRotState());
        telemetry.update();
    }
    private void setServoPositions(double clawOCPos, double clawRotPos, double headRotPos) {
        clawOC.setPosition(calculateServoPosition(clawOCPos, CLAW_OFFSET));
        clawRot.setPosition(calculateServoPosition(clawRotPos, CLAW_ROT_OFFSET));
        headRot.setPosition(calculateServoPosition(headRotPos, HEAD_ROT_OFFSET));
    }
    private double calculateServoPosition(double input, double offset) {
        return (input + offset) / MAX_SERVO_ROTATION;
    }
}
