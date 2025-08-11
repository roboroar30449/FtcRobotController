package org.firstinspires.ftc.teamcode.robot;

import com.pedropathing.localization.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class RobotHardware {

    // Define all motors, servos and limit switches
    public DcMotor LFMotor, LBMotor, RFMotor, RBMotor, pivotRot, armL, armR;
    public Servo clawOC, clawRot, headRot;
    public GoBildaPinpointDriver pinpoint;
    public IMU imu;

    public Telemetry telemetry;
    public RobotHardware(HardwareMap hwMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        LFMotor = hwMap.get(DcMotor.class, "LFMotor"); //E0
        RFMotor = hwMap.get(DcMotor.class, "RFMotor"); //C0
        LBMotor = hwMap.get(DcMotor.class, "LBMotor"); //E1
        RBMotor = hwMap.get(DcMotor.class, "RBMotor"); //C1

        pivotRot = hwMap.get(DcMotor.class, "PivotRot"); //C3
        armL = hwMap.get(DcMotor.class, "ArmL"); //E2
        armR = hwMap.get(DcMotor.class, "ArmR"); //C2

        clawOC = hwMap.get(Servo.class, "ClawOC"); //C0
        clawRot = hwMap.get(Servo.class, "ClawRot"); //C1
        headRot = hwMap.get(Servo.class, "HeadRot"); //C2

        pinpoint = hwMap.get(GoBildaPinpointDriver.class, "pinpoint"); //I2CBus1
        imu = hwMap.get(IMU.class, "imu"); //I2CBus0

        setMotorDirections();
        initMotorModes();
    }
    private void setMotorDirections() {
        LFMotor.setDirection(DcMotor.Direction.REVERSE);
        LBMotor.setDirection(DcMotor.Direction.REVERSE);
        RFMotor.setDirection(DcMotor.Direction.FORWARD);
        RBMotor.setDirection(DcMotor.Direction.FORWARD);
        pivotRot.setDirection(DcMotor.Direction.REVERSE);
        armL.setDirection(DcMotor.Direction.REVERSE);
        armR.setDirection(DcMotor.Direction.FORWARD);
        LFMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LBMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RFMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RBMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pivotRot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    private void initMotorModes() {
        for (DcMotor motor : new DcMotor[]{armR, armL, pivotRot}) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
}