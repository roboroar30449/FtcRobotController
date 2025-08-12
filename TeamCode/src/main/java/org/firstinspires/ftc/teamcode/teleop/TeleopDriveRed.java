package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.robot.MechController;
import org.firstinspires.ftc.teamcode.robot.MechState;
import org.firstinspires.ftc.teamcode.robot.RobotHardware;
@TeleOp(name = "TeleOp Drive Red", group = "Red OpModes")
public class TeleopDriveRed extends LinearOpMode {
    public double drive, strafe, turn;
    public double arm, claw, head, pivot;
    RobotHardware robot;
    MechController mechController;
    boolean buttonPressed = false; // To handle button press logic

    @Override
    public void runOpMode() {
        robot = new RobotHardware(hardwareMap, telemetry);
        mechController = new MechController(robot);
        mechController.handleMechState(MechState.IDLE_POSITION);
        mechController.allTelemetry();

        waitForStart();

        while (opModeIsActive()) {
            drive = -gamepad1.left_stick_y;
            strafe = gamepad1.left_stick_x;
            turn = gamepad1.right_stick_x;
            driveRobot(drive, strafe, turn);

            arm = -gamepad2.left_stick_y;
            pivot = gamepad2.left_stick_x;
            head = gamepad2.right_stick_y;
            claw = gamepad2.right_stick_x;
            driveMech(arm, claw, head, pivot);

            // Check for button presses and handle them
            if (gamepad1.a && !buttonPressed) {
                buttonPressed = true;
                mechController.handleMechState(MechState.IDLE_POSITION);
                while(mechController.isBusy() && opModeIsActive()) {
                    mechController.allTelemetry();
                    idle();
                }
            }

            if (gamepad1.x && !buttonPressed) {
                buttonPressed = true;
                mechController.handleMechState(MechState.SUB_POSITION);
                waitForStateToFinish();
                mechController.handleMechState(MechState.CLAW_CLOSE);
                waitForServoStateToFinish();
                sleep(100);
                mechController.handleMechState(MechState.RESET_POSITION);
                waitForStateToFinish();
            }

            if (gamepad1.y && !buttonPressed) {
                buttonPressed = true;
                mechController.handleMechState(MechState.HIGH_BASKET_POSITION);
                mechController.openClaw();
                mechController.handleMechState(MechState.RESET_POSITION);
            }

            if (gamepad1.a && !buttonPressed) {
                buttonPressed = true;
                mechController.handleMechState(MechState.ENDGAME_POSITION);
            }

            if ((gamepad1.right_trigger > 0.2 || gamepad2.right_trigger > 0.2) && !buttonPressed) {
                buttonPressed = true;
                mechController.openClaw();
            }

            // Reset button press flag when no button is pressed
            if (!gamepad1.x && !gamepad1.y && !gamepad1.a && gamepad1.right_trigger <= 0.2 && gamepad2.right_trigger <= 0.2) {
                buttonPressed = false;
            }

            // Call telemetry once per loop cycle
            mechController.allTelemetry();
        }
    }
    public void driveRobot(double drive, double strafe, double turn) {
        double theta = Math.atan2(drive, strafe);
        double power = Math.hypot(strafe, drive);
        double sin = Math.sin(theta - Math.PI / 4);
        double cos = Math.cos(theta - Math.PI / 4);
        double maxsc = Math.max(Math.abs(sin), Math.abs(cos));
        double lfm = ((power * cos) / maxsc) + turn;
        double lbm = ((power * sin) / maxsc) + turn;
        double rfm = ((power * sin) / maxsc) - turn;
        double rbm = ((power * cos) / maxsc) - turn;
        double max = Math.max(Math.abs(lfm), (Math.max(Math.abs(lbm), (Math.max(Math.abs(rfm), (Math.max(Math.abs(rbm), 1.0)))))));

        robot.LFMotor.setPower(lfm / max);
        robot.RFMotor.setPower(rfm / max);
        robot.LBMotor.setPower(lbm / max);
        robot.RBMotor.setPower(rbm / max);
    }

    public void driveMech(double arm, double claw, double head, double pivot) {
        robot.armR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.armL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Arm control
        if (mechController.ArmState() < (MechController.armMinLimit)) { // Arm Min Limit
            robot.armR.setPower(0);
            robot.armL.setPower(0);
            double armTicks = mechController.CalculateArmTicks(MechController.armMinLimit);
            for (DcMotor arms : new DcMotor[]{robot.armL, robot.armR}) {
                arms.setPower(mechController.motorPower);
                arms.setTargetPosition((int) armTicks);
                arms.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
        } else if (mechController.ArmState() > (MechController.armMaxLimit)) { // Arm Max Limit
            robot.armR.setPower(0);
            robot.armL.setPower(0);
            double armTicks = mechController.CalculateArmTicks(MechController.armMaxLimit);
            for (DcMotor arms : new DcMotor[]{robot.armL, robot.armR}) {
                arms.setPower(mechController.motorPower);
                arms.setTargetPosition((int) armTicks);
                arms.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
        } else {
            robot.armR.setPower(arm);
            robot.armL.setPower(arm);
        }

        // Pivot control
        if (mechController.PivotState() < (MechController.pivotMinLimit)) { // Pivot Min Limit
            robot.pivotRot.setPower(0);
            double pivotTicks = mechController.CalculatePivotTicks(MechController.pivotMinLimit);
            robot.pivotRot.setPower(mechController.motorPower);
            robot.pivotRot.setTargetPosition((int) pivotTicks);
            robot.pivotRot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        } else if (mechController.PivotState() > (MechController.pivotMaxLimit)) { // Pivot Max Limit
            robot.pivotRot.setPower(0);
            double pivotTicks = mechController.CalculatePivotTicks(MechController.pivotMaxLimit);
            robot.pivotRot.setPower(mechController.motorPower);
            robot.pivotRot.setTargetPosition((int) pivotTicks);
            robot.pivotRot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        } else {
            robot.pivotRot.setPower(pivot);
        }

        // Claw control
        double clawPos = (claw + 1.0) / 2.0; // -1.0 to 1.0 -> 0.0 to 1.0
        clawPos = Math.max(((MechController.clawRotOffset - MechController.clawRotLimit) / MechController.MAX_SERVO_ROTATION), Math.min(((MechController.clawRotOffset + MechController.clawRotLimit) / MechController.MAX_SERVO_ROTATION), clawPos)); // Set min and max as -90 to 90 deg
        robot.clawRot.setPosition(clawPos);

        // Head control
        double headPos = (head + 1.0) / 2.0; // -1.0 to 1.0 -> 0.0 to 1.0
        headPos = Math.max(((MechController.headRotOffset + MechController.headMinLimit) / MechController.MAX_SERVO_ROTATION), Math.min(((MechController.headRotOffset + MechController.headMaxLimit) / MechController.MAX_SERVO_ROTATION), headPos)); // Set min and max as 0 to 115 deg
        robot.headRot.setPosition(headPos);
    }
    private void waitForStateToFinish() {
        while (mechController.isBusy() && opModeIsActive()) {
            mechController.allTelemetry();
            idle();
        }
    }
    private void waitForServoStateToFinish() {
        while (mechController.isServoBusy() && opModeIsActive()) {
            mechController.allTelemetry();
            idle();
        }
    }
}
