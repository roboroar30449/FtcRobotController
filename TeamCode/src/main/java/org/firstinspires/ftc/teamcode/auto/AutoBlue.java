package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.robot.MechController;
import org.firstinspires.ftc.teamcode.robot.MechState;
import org.firstinspires.ftc.teamcode.robot.RobotHardware;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;

@Autonomous(name="Auto Drive Blue", group="Blue OpModes")
public class AutoBlue extends OpMode {
    private final Pose startPose = new Pose(8, 80, Math.toRadians(0));  // Starting position
    private final Pose highBasketPose = new Pose(20, 120, Math.toRadians(-45)); // Scoring position
    private final Pose pickup1Pose = new Pose(30, 120, Math.toRadians(0)); // First sample pickup
    private final Pose pickup2Pose = new Pose(30, 130, Math.toRadians(0)); // Second sample pickup
    private final Pose pickup3Pose = new Pose(30, 130, Math.toRadians(45)); // Third sample pickup
    private final Pose subPickupPose = new Pose(71, 105, Math.toRadians(-90)); // Sub sample pickup
    private final Pose endGamePose = new Pose(70, 100, Math.toRadians(90)); // Parking position

    private Path scorePreload;
    private PathChain grabPickup1, grabPickup2, grabPickup3, grabSubPickup, scorePickup1, scorePickup2, scorePickup3, scoreSubPickup, park;
    private Follower follower;
    RobotHardware robot;
    MechController mechController;
    private Timer pathTimer;
    private int pathState = 0;
    public void buildPaths() {
        // Path for scoring preload
        scorePreload = new Path(new BezierLine(new Point(startPose), new Point(highBasketPose)));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), highBasketPose.getHeading());

        // Path chains for picking up and scoring samples
        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(highBasketPose), new Point(pickup1Pose)))
                .setLinearHeadingInterpolation(highBasketPose.getHeading(), pickup1Pose.getHeading())
                .build();

        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup1Pose), new Point(highBasketPose)))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), highBasketPose.getHeading())
                .build();

        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(highBasketPose), new Point(pickup2Pose)))
                .setLinearHeadingInterpolation(highBasketPose.getHeading(), pickup2Pose.getHeading())
                .build();

        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup2Pose), new Point(highBasketPose)))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), highBasketPose.getHeading())
                .build();

        grabPickup3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(highBasketPose), new Point(pickup3Pose)))
                .setLinearHeadingInterpolation(highBasketPose.getHeading(), pickup3Pose.getHeading())
                .build();

        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup3Pose), new Point(highBasketPose)))
                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), highBasketPose.getHeading())
                .build();

        grabSubPickup = follower.pathBuilder()
                .addPath(new BezierLine(new Point(highBasketPose), new Point(subPickupPose)))
                .setLinearHeadingInterpolation(highBasketPose.getHeading(), subPickupPose.getHeading())
                .build();

        scoreSubPickup = follower.pathBuilder()
                .addPath(new BezierLine(new Point(subPickupPose), new Point(highBasketPose)))
                .setLinearHeadingInterpolation(subPickupPose.getHeading(), highBasketPose.getHeading())
                .build();

        park  = follower.pathBuilder()
                .addPath(new BezierLine(new Point(highBasketPose), new Point(endGamePose)))
                .setLinearHeadingInterpolation(highBasketPose.getHeading(), endGamePose.getHeading())
                .build();
    }
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0: // Move from start to scoring position
                follower.followPath(scorePreload);
                mechController.handleMechState(MechState.HIGH_BASKET_POSITION);
                setPathState(1);
                break;

            case 1: // Wait until the robot is near the scoring position
                if (!follower.isBusy()) {
                    follower.followPath(grabPickup1, true);
                    mechController.handleMechState(MechState.COLLECTING_PS_POSITION);
                    setPathState(2);
                }
                break;

            case 2: // Wait until the robot is near the first sample pickup position
                if (!follower.isBusy()) {
                    follower.followPath(scorePickup1, true);
                    mechController.handleMechState(MechState.HIGH_BASKET_POSITION);
                    setPathState(3);
                }
                break;

            case 3: // Wait until the robot returns to the scoring position
                if (!follower.isBusy()) {
                    follower.followPath(grabPickup2, true);
                    mechController.handleMechState(MechState.COLLECTING_PS_POSITION);
                    setPathState(4);
                }
                break;

            case 4: // Wait until the robot is near the second sample pickup position
                if (!follower.isBusy()) {
                    follower.followPath(scorePickup2, true);
                    mechController.handleMechState(MechState.HIGH_BASKET_POSITION);
                    setPathState(5);
                }
                break;

            case 5: // Wait until the robot returns to the scoring position
                if (!follower.isBusy()) {
                    follower.followPath(grabPickup3, true);
                    mechController.handleMechState(MechState.COLLECTING_PS_POSITION);
                    setPathState(6);
                }
                break;

            case 6: // Wait until the robot is near the third sample pickup position
                if (!follower.isBusy()) {
                    follower.followPath(scorePickup3, true);
                    mechController.handleMechState(MechState.HIGH_BASKET_POSITION);
                    setPathState(7);
                }
                break;

            case 7: // Wait until the robot returns to the scoring position
                if (!follower.isBusy()) {
                    follower.followPath(grabSubPickup, true);
                    mechController.handleMechState(MechState.SUB_POSITION);
                    setPathState(8);
                }
                break;

            case 8: // Wait until the robot is near the sub sample pickup position
                if (!follower.isBusy()) {
                    follower.followPath(scoreSubPickup, true);
                    mechController.handleMechState(MechState.HIGH_BASKET_POSITION);
                    setPathState(9);
                }
                break;

            case 9: // Wait until the robot returns to the scoring position
                if (!follower.isBusy()) {
                    follower.followPath(park, true);
                    mechController.handleMechState(MechState.ENDGAME_POSITION);
                    setPathState(10);
                }
                break;

            case 10: // Wait until the robot is near the parking position
                if (!follower.isBusy()) {
                    setPathState(-1); // End the autonomous routine
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