package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "DC Motor Test", group = "Testing")
public class DCTestRel extends LinearOpMode {

    private DcMotor armR;
    private static final double TICKS_PER_ROTATION = 1425.1; //312 RPM = 537.7 | 117 RPM = 1425.1
    private boolean buttonPressed = false;

    @Override
    public void runOpMode() {
        armR = hardwareMap.get(DcMotor.class, "ArmR");

        armR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armR.setDirection(DcMotor.Direction.FORWARD);
        armR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            handleManualMovement();

            telemetry.addData("ArmR Position", Math.round((armR.getCurrentPosition()/TICKS_PER_ROTATION)*120));
            telemetry.update();

            sleep(50);
        }
    }

    private void handleManualMovement() {
        if (!buttonPressed) {
            if (gamepad1.x) {
                moveArmRRelative((int) TICKS_PER_ROTATION); //Clock-wise
                buttonPressed = true;
            } else if (gamepad1.y) {
                moveArmRRelative((int) -TICKS_PER_ROTATION); //Anti Clock-wise
                buttonPressed = true;
            }
        } else if (!gamepad1.x && !gamepad1.y) {
            buttonPressed = false;
        }
    }

    private void moveArmRRelative(int ticks) {
        int currentPos = armR.getCurrentPosition();
        int targetPos = currentPos + ticks;

        armR.setTargetPosition(targetPos);
        armR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armR.setPower(0.1);
    }
}
