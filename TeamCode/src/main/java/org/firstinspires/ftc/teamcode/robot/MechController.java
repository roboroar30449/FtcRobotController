package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class MechController {
    private final RobotHardware robot;
    private final Telemetry telemetry;

    MechState currentState;

    //Hardware constants
    public double motorPower = 0.25;
    public static final double MAX_SERVO_ROTATION = 300.0;//Deg
    public static final double ARM_TICKS_PER_FULL_ROTATION = 537.7;//Encoder Resolution PPR for RPM 312
    public static final double PIVOT_TICKS_PER_FULL_ROTATION = 1425.1;//Encoder Resolution PPR for RPM 117
    public static final double DISTANCE_PER_ROTATION = 120.0;//Pitch * Tooth
    public static final double PIVOT_GEAR_RATIO = 4.0;//Pivot Gear Ratio

    //Limit constants
    public static final double armMinLimit = 0; // Arm Min Limit
    public static final double armMaxLimit = 980; // Arm Max Limit
    public static final double pivotMinLimit = 0; // Pivot Min Limit
    public static final double pivotMaxLimit = 100; // Pivot Max Limit
    public static final double clawOCMinLimit = 0; // ClawOC Min Limit (68.2)
    public static final double clawOCMaxLimit = 77; // ClawOC Max Limit (135.7)
    public static final double clawRotLimit = 90; // Claw Rot Limit
    public static final double headMinLimit = 0; // Head Min Limit (37)
    public static final double headMaxLimit = 135; // Head Max Limit (198)

    //Offset constants
    public static final double clawOffset = 60;//Deg
    public static final double clawRotOffset = 150;//Deg
    public static final double headRotOffset = 40;//Deg

    //Status
    public static boolean clawOCStatus = false;

    public MechController(RobotHardware RoboRoar) {
        this.robot = RoboRoar;
        this.telemetry = RoboRoar.telemetry;
    }

    public double CalculateServoPosition(double target, double offset) {
        return ((target + offset) / MAX_SERVO_ROTATION);
    }

    public double CalculatePivotTicks(double degrees) {
        return (PIVOT_TICKS_PER_FULL_ROTATION / 360.0) * degrees;
    }

    public double CalculateArmTicks(double travelDistance) {
        double numOfRotations = travelDistance / DISTANCE_PER_ROTATION;
        return numOfRotations * ARM_TICKS_PER_FULL_ROTATION;
    }

    public double ClawOCState() {
        return ((robot.clawOC.getPosition() * MAX_SERVO_ROTATION) - clawOffset);
    }

    public double ClawRotState() {
        return ((robot.clawRot.getPosition() * MAX_SERVO_ROTATION) - clawRotOffset);
    }

    public double HeadRotState() {
        return ((robot.headRot.getPosition() * MAX_SERVO_ROTATION) - headRotOffset);
    }

    public double ArmState() {
        return (robot.armR.getCurrentPosition() / ARM_TICKS_PER_FULL_ROTATION * DISTANCE_PER_ROTATION);
    }

    public double PivotState() {
        return (robot.pivotRot.getCurrentPosition() / (PIVOT_TICKS_PER_FULL_ROTATION / 360));
    }

    public void handleMechState(MechState state) {
        switch (state) {
            case IDLE_POSITION:
                movePivotAndArms(0, 0);
                setClawAndHead(0, headMaxLimit - 55);
                closeClaw();
                currentState = MechState.IDLE_POSITION;
                break;

            case HIGH_BASKET_POSITION:
                movePivotAndArms(5, 90);
                setClawAndHead(0, 0);
                closeClaw();
                currentState = MechState.HIGH_BASKET_POSITION;
                break;

            case ENDGAME_POSITION:
                movePivotAndArms(5, 244.8);
                setClawAndHead(90, 0);
                openClaw();
                currentState = MechState.ENDGAME_POSITION;
                break;

            case RESET_POSITION:
                movePivotAndArms(0, 0);
                setClawAndHead(0, headMaxLimit - 35);
                closeClaw();
                currentState = MechState.RESET_POSITION;
                break;

            case SUB_POSITION:
                movePivotAndArms(0, 100);
                setClawAndHead(0, headMaxLimit);
                openClaw();
                currentState = MechState.SUB_POSITION;
                break;

            case COLLECTING_PS_POSITION:
                movePivotAndArms(0, 0);
                setClawAndHead(0, headMaxLimit);
                openClaw();
                currentState = MechState.COLLECTING_PS_POSITION;
                break;

            case CLAW_CLOSE:
                closeClaw();
                currentState = MechState.CLAW_CLOSE;
                break;

            case CLAW_OPEN:
                openClaw();
                currentState = MechState.CLAW_OPEN;
                break;
        }
    }


    private void setClawAndHead(double clawRot, double headRot) {
        robot.clawRot.setPosition(CalculateServoPosition(clawRot, clawRotOffset));
        robot.headRot.setPosition(CalculateServoPosition(headRot, headRotOffset));
    }

    public boolean isBusy() {
        if (robot.armL.isBusy() || robot.armR.isBusy() || robot.pivotRot.isBusy()){
            return true;
        } else {
            return false;
        }
    }

    public boolean isServoBusy() {
        if (ClawOCState() <= clawOCMaxLimit){
            return true;
        } else {
            return false;
        }
    }
    private void movePivotAndArms(double targetDeg, double targetPosition) {
        double pivotTicks = CalculatePivotTicks(targetDeg) * PIVOT_GEAR_RATIO;
        robot.pivotRot.setPower(motorPower);
        robot.pivotRot.setTargetPosition((int) pivotTicks);
        robot.pivotRot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        double armTicks = CalculateArmTicks(targetPosition);
        for (DcMotor arm : new DcMotor[]{robot.armL, robot.armR}) {
            arm.setPower(motorPower);
            arm.setTargetPosition((int) armTicks);
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }

    public void allTelemetry() {
        String clawStatus;
        if (ClawOCState() > (clawOCMaxLimit-2)) {
            clawStatus = "Close";
        } else {
            clawStatus = "Open";
        }
        telemetry.addData("State", currentState + " | Busy: " + isBusy());
        telemetry.addData("Position X", Math.round(robot.pinpoint.getPosX()) + " | Position Y: " + Math.round(robot.pinpoint.getPosY()) + " | Heading: " + Math.round(robot.pinpoint.getHeading()));
        telemetry.addData("Claw", clawStatus + " | Claw Pos: " + Math.round(ClawRotState()) + " | Head Pos: " + Math.round(HeadRotState()));
        telemetry.addData("Arm Pos in mm", Math.round(ArmState()) + " | Pivot Pos: " + Math.round(PivotState() / PIVOT_GEAR_RATIO));
        robot.pinpoint.update();
        telemetry.update();
    }
    public void openClaw() {
        robot.clawOC.setPosition(CalculateServoPosition(clawOCMinLimit, clawOffset)); //Claw Open
            clawOCStatus = true;
        }
    public void closeClaw() {
            robot.clawOC.setPosition(CalculateServoPosition(clawOCMaxLimit, clawOffset)); //Claw Close
            clawOCStatus = false;
        }
    }