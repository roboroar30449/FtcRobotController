package org.firstinspires.ftc.teamcode.robot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.robot.Robot;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class MechController {
    private final RobotHardware robot;
    private final Telemetry telemetry;

    MechState currentState;
    private double motorPower = 0.1;
    public static final double MAX_SERVO_ROTATION = 300.0;//Deg
    public static final double ARM_TICKS_PER_FULL_ROTATION = 537.7;//Encoder Resolution PPR for RPM 312
    public static final double PIVOT_TICKS_PER_FULL_ROTATION = 1425.1;//Encoder Resolution PPR for RPM 117
    public static final double DISTANCE_PER_ROTATION = 120.0;//Pitch * Tooth

    public final double clawOffset = 30;//Deg
    private final double clawRotOffset = 150;//Deg
    private final double headRotOffset = 30;//Deg

    public MechController(RobotHardware RoboRoar) {
        this.robot = RoboRoar;
        this.telemetry = RoboRoar.telemetry;
    }

    public double CalculateServoPosition(double target, double offset) {
        return ((target + offset) / MAX_SERVO_ROTATION);
    }
    private double CalculatePivotTicks(double degrees) {
        return (PIVOT_TICKS_PER_FULL_ROTATION / 360.0) * degrees;
    }
    private double CalculateArmTicks(double travelDistance) {
        double numOfRotations = travelDistance / DISTANCE_PER_ROTATION;
        return numOfRotations * ARM_TICKS_PER_FULL_ROTATION;
    }
    public double ClawOCState(){
        return ((robot.clawOC.getPosition()*MAX_SERVO_ROTATION)-clawOffset);
    }
    public double ClawRotState() {
        return ((robot.clawRot.getPosition()*MAX_SERVO_ROTATION)-clawRotOffset);
    }
    public double HeadRotState() {
        return ((robot.headRot.getPosition()*MAX_SERVO_ROTATION)-headRotOffset);
    }
    public double ArmState(){
        return (robot.armR.getCurrentPosition()/ARM_TICKS_PER_FULL_ROTATION*DISTANCE_PER_ROTATION);
    }
    public double PivotState(){
        return (robot.pivotRot.getCurrentPosition()/(PIVOT_TICKS_PER_FULL_ROTATION/360));
    }
    public void handleMechState(MechState state) {
        switch (state) {
            case IDLE_POSITION:
                if (!isBusy()) {
                    movePivotAndArms(0, 0);
                    setClawAndHead(60,0,0);
                    currentState = MechState.IDLE_POSITION;
                }
                break;

            case HIGH_BASKET_POSITION:
                if (!isBusy()) {
                    movePivotAndArms(100, 979.2);
                    setClawAndHead(60, 0, 0);
                    currentState = MechState.HIGH_BASKET_POSITION;
                }
                break;

            case ENDGAME_POSITION:
                if (!isBusy()) {
                    movePivotAndArms(100, 244.8);
                    setClawAndHead(0, 90, 0);
                    currentState = MechState.ENDGAME_POSITION;
                }
                break;

            case RESET_POSITION:
                if (!isBusy()) {
                    movePivotAndArms(0, 0);
                    setClawAndHead(60, 0, 55);
                    currentState = MechState.RESET_POSITION;
                }
                break;

            case SUB_POSITION:
                if (!isBusy()) {
                    movePivotAndArms(0, 489.6);
                    setClawAndHead(0, 0, 40);
                    currentState = MechState.SUB_POSITION;
                }
                break;

            case COLLECTING_PS_POSITION:
                if (!isBusy()) {
                    movePivotAndArms(0, 0);
                    setClawAndHead(0, 0, 115);
                    currentState = MechState.COLLECTING_PS_POSITION;
                }
                break;

            case LOW_BASKET_POSITION:
                if (!isBusy()) {
                    // TODO: Fill in these states
                    currentState = MechState.LOW_BASKET_POSITION;
                }
                break;

            case SPECIMEN_POSITION:
                if (!isBusy()) {
                    // TODO: Fill in these states
                    currentState = MechState.SPECIMEN_POSITION;
                }
                break;
        }
    }
    private void setClawAndHead(double clawOC, double clawRot, double headRot) {
        robot.clawOC.setPosition(CalculateServoPosition(clawOC, clawOffset));
        robot.clawRot.setPosition(CalculateServoPosition(clawRot, clawRotOffset));
        robot.headRot.setPosition(CalculateServoPosition(headRot, headRotOffset));
    }
    public boolean isBusy() {
        if (robot.armL.isBusy() || robot.armR.isBusy() ||  robot.pivotRot.isBusy()){
            return true;
        } else {
            return false;
        }
    }
    private void movePivotAndArms(double targetDeg, double targetPosition) {
        double pivotTicks = CalculatePivotTicks(targetDeg);
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
    public void allTelemetry(){
        telemetry.addData("Current State", currentState + " | Busy: " + isBusy());
        telemetry.addData("Position X", Math.round(robot.pinpoint.getPosX()) + " | Position Y: " + Math.round(robot.pinpoint.getPosY()) + " | Heading: " + Math.round(robot.pinpoint.getHeading()));
        telemetry.addData("Claw OC", Math.round(ClawOCState()) + " | Claw Position: " + Math.round(ClawRotState()) + " | Head Position: " + Math.round(HeadRotState()));
        telemetry.addData("Arm Position in mm", Math.round(ArmState()) + " | Pivot Position: " + Math.round(PivotState()));
        telemetry.update();
    }

    public void toggleClaw() {
        if (ClawOCState() > 30){
            robot.clawOC.setPosition(CalculateServoPosition(0, clawOffset));
        } else {
            robot.clawOC.setPosition(CalculateServoPosition(60, clawOffset));
        }
    }
}