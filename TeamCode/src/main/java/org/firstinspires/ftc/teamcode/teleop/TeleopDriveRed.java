package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.MechController;
import org.firstinspires.ftc.teamcode.robot.MechState;
import org.firstinspires.ftc.teamcode.robot.RobotHardware;

@TeleOp(name = "TeleOp Drive Red", group = "Red OpModes")
public class TeleopDriveRed extends LinearOpMode {

    RobotHardware robot;
    MechController mechController;
    MechState currentState;

    @Override
    public void runOpMode() {
        robot = new RobotHardware(hardwareMap, telemetry);
        mechController = new MechController(robot);
        mechController.handleMechState(MechState.IDLE_POSITION);
        mechController.allTelemetry();

        waitForStart();

        while (opModeIsActive()) {
            double drive = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double turn = gamepad1.right_stick_x;
            double max = Math.max(Math.abs(drive) + Math.abs(turn) + Math.abs(strafe), 1);
            driveRobot(drive, strafe, turn, max);

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

            if (gamepad1.y) {
                mechController.handleMechState(MechState.HIGH_BASKET_POSITION);
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

            if (gamepad1.a) {
                mechController.handleMechState(MechState.ENDGAME_POSITION);
            }

            if (gamepad1.b) {
                mechController.toggleClaw();
            }
            mechController.allTelemetry();
        }
    }

    public void driveRobot(double drive, double strafe, double turn, double max) {
        robot.LFMotor.setPower((drive + turn + strafe) / max);
        robot.RFMotor.setPower((drive - turn - strafe) / max);
        robot.LBMotor.setPower((drive + turn - strafe) / max);
        robot.RBMotor.setPower((drive - turn + strafe) / max);
    }
}
