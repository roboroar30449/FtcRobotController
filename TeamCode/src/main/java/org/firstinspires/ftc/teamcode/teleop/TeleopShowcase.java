package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.robot.MechController;
import org.firstinspires.ftc.teamcode.robot.MechState;
import org.firstinspires.ftc.teamcode.robot.RobotHardware;

@TeleOp(name = "Teleop Showcase", group = "Red OpModes")
public class TeleopShowcase extends LinearOpMode {
    public double drive, strafe, turn;
    public double arm, claw, head, pivot;
    RobotHardware robot;
    MechController mechController;

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
            claw = gamepad2.left_stick_x;
            head = gamepad2.right_stick_y;
            pivot = gamepad2.right_stick_x;
            driveMech(arm, claw, head, pivot);

            if (gamepad1.x) {
                mechController.handleMechState(MechState.SUB_POSITION);
                while (mechController.isBusy() && opModeIsActive()) {
                    mechController.allTelemetry();
                    idle();
                }
                mechController.toggleClaw();
                mechController.handleMechState(MechState.RESET_POSITION);
                while (mechController.isBusy() && opModeIsActive()) {
                    mechController.allTelemetry();
                    idle();
                }
            }

//            if (gamepad1.y) {
//                mechController.handleMechState(MechState.HIGH_BASKET_POSITION);
//                while (mechController.isBusy() && opModeIsActive()) {
//                    mechController.allTelemetry();
//                    idle();
//                }
//                mechController.toggleClaw();
//                mechController.handleMechState(MechState.RESET_POSITION);
//                while (mechController.isBusy() && opModeIsActive()) {
//                    mechController.allTelemetry();
//                    idle();
//                }
//            }

//            if (gamepad1.a) {
//                mechController.handleMechState(MechState.ENDGAME_POSITION);
//            }
            if (gamepad2.a) {
                robot.clawRot.setPosition(MechController.clawRotOffset / MechController.MAX_SERVO_ROTATION); // Set Claw Rotation Position to 0 Deg
            }
            if (gamepad1.b || gamepad2.b) {
                mechController.toggleClaw();
            }
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

        double speedMultiplier = 0.5; // 50% of max speed

        robot.LFMotor.setPower((lfm / max) * speedMultiplier);
        robot.RFMotor.setPower((rfm / max) * speedMultiplier);
        robot.LBMotor.setPower((lbm / max) * speedMultiplier);
        robot.RBMotor.setPower((rbm / max) * speedMultiplier);
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
}
