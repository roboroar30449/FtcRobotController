package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;
import org.firstinspires.ftc.teamcode.robot.MechController;
import org.firstinspires.ftc.teamcode.robot.MechState;
import org.firstinspires.ftc.teamcode.robot.RobotHardware;

@Autonomous(name="Auto Square", group="OpModes")
public class AutoSquare extends OpMode {
    private final Pose startPose = new Pose(120, 24, Math.toRadians(0));  // Starting position
    private final Pose stop1 = new Pose(96, 24, Math.toRadians(0)); // Scoring position
    private final Pose stop2 = new Pose(96, 48, Math.toRadians(-90)); // First sample pickup
    private final Pose stop3 = new Pose(120, 48, Math.toRadians(-90)); // Second sample pickup

    private Path moveToStop1;
    private PathChain moveToStop2, moveToStop3, moveToStart;
    private Follower follower;
    RobotHardware robot;
    MechController mechController;
    private Timer pathTimer;
    private int pathState = 0;
    public void buildPaths() {
        // Path for scoring preload
        moveToStop1 = new Path(new BezierLine(new Point(startPose), new Point(stop1)));
        moveToStop1.setLinearHeadingInterpolation(startPose.getHeading(), stop1.getHeading());

        // Path chains for picking up and scoring samples
        moveToStop2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(stop1), new Point(stop2)))
                .setLinearHeadingInterpolation(stop1.getHeading(), stop2.getHeading())
                .build();

        moveToStop3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(stop2), new Point(stop3)))
                .setLinearHeadingInterpolation(stop2.getHeading(), stop3.getHeading())
                .build();

        moveToStart = follower.pathBuilder()
                .addPath(new BezierLine(new Point(stop3), new Point(startPose)))
                .setLinearHeadingInterpolation(stop3.getHeading(), startPose.getHeading())
                .build();

    }
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0: // Move from start to scoring position
                follower.followPath(moveToStop1);
                setPathState(1);
                break;

            case 1: // Wait until the robot is near the scoring position
                if (!follower.isBusy()) {
                    follower.followPath(moveToStop2, true);
                    setPathState(2);
                }
                break;

            case 2: // Wait until the robot is near the first sample pickup position
                if (!follower.isBusy()) {
                    follower.followPath(moveToStop3, true);
                    setPathState(3);
                }
                break;

            case 3: // Wait until the robot returns to the scoring position
                if (!follower.isBusy()) {
                    follower.followPath(moveToStart, true);
                    setPathState(4);
                }
                break;
        }
    }

    public void setPathState(int nextState) {
        pathState = nextState;
        if (pathTimer != null) pathTimer.resetTimer();
    }
    @Override
    public void init() {
        pathTimer = new Timer();
        Constants.setConstants(FConstants.class, LConstants.class);
        robot = new RobotHardware(hardwareMap, telemetry);
        mechController = new MechController(robot);
        mechController.handleMechState(MechState.IDLE_POSITION);
        mechController.allTelemetry();
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(startPose);
        buildPaths();
    }
    @Override
    public void loop() {
        if (pathState == -1) return;
        follower.update();
        autonomousPathUpdate();
        telemetry.addData("Path State", pathState);
        telemetry.addData("Position", follower.getPose().toString());
        telemetry.addData("Time in State", pathTimer.getElapsedTime());
        telemetry.update();
    }
}