//package pedroPathing.examples;
//
//import com.pedropathing.localization.Pose;
//import com.pedropathing.pathgen.BezierCurve;
//import com.pedropathing.pathgen.BezierLine;
//import com.pedropathing.pathgen.Path;
//import com.pedropathing.pathgen.PathChain;
//import com.pedropathing.pathgen.Point;
//
//private final Pose startPose = new Pose(135, 50, Math.toRadians(0));  // Starting/idle position
//private final Pose dropPose = new Pose(125, 25, Math.toRadians(135)); // Scoring/high basket position
//
//private final Pose preStaged1Pose = new Pose(115, 25, Math.toRadians(180)); // First Pre-staged sample pickup
//private final Pose preStaged2Pose = new Pose(115, 15, Math.toRadians(180)); // Second Pre-staged sample pickup
//private final Pose preStaged3Pose = new Pose(115, 12, Math.toRadians(215)); // Third Pre-staged sample pickup
//private final Pose subPose = new Pose(72, 40, Math.toRadians(90)); // sub pickup/position
//private final Pose endPose = new Pose(72, 45, Math.toRadians(-90));    // Endgame position - low rung
//
//        private Path scorePreload, park;
//        private PathChain grabPickup1, grabPickup2, grabPickup3, scorePickup1, scorePickup2, scorePickup3;
//
//public void buildPaths() {
//    // Path for scoring preload
//    scorePreload = new Path(new BezierLine(new Point(startPose), new Point(scorePose)));
//    scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());
//
//    // Path chains for picking up and scoring samples
//    grabPickup1 = follower.pathBuilder()
//            .addPath(new BezierLine(new Point(scorePose), new Point(pickup1Pose)))
//            .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading())
//            .build();
//
//    scorePickup1 = follower.pathBuilder()
//            .addPath(new BezierLine(new Point(pickup1Pose), new Point(scorePose)))
//            .setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePose.getHeading())
//            .build();
//
//    grabPickup2 = follower.pathBuilder()
//            .addPath(new BezierLine(new Point(scorePose), new Point(pickup2Pose)))
//            .setLinearHeadingInterpolation(scorePose.getHeading(), pickup2Pose.getHeading())
//            .build();
//
//    scorePickup2 = follower.pathBuilder()
//            .addPath(new BezierLine(new Point(pickup2Pose), new Point(scorePose)))
//            .setLinearHeadingInterpolation(pickup2Pose.getHeading(), scorePose.getHeading())
//            .build();
//
//    grabPickup3 = follower.pathBuilder()
//            .addPath(new BezierLine(new Point(scorePose), new Point(pickup3Pose)))
//            .setLinearHeadingInterpolation(scorePose.getHeading(), pickup3Pose.getHeading())
//            .build();
//
//    scorePickup3 = follower.pathBuilder()
//            .addPath(new BezierLine(new Point(pickup3Pose), new Point(scorePose)))
//            .setLinearHeadingInterpolation(pickup3Pose.getHeading(), scorePose.getHeading())
//            .build();
//
//    // Curved path for parking
//    park = new Path(new BezierCurve(new Point(scorePose), new Point(parkControlPose), new Point(parkPose)));
//    park.setLinearHeadingInterpolation(scorePose.getHeading(), parkPose.getHeading());
//}

package org.firstinspires.ftc.teamcode.auto;
/*
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;

public class AutoRed {
    private final Pose startPose = new Pose(135, 50, Math.toRadians(0));  // Starting/idle position
    private final Pose scorePose = new Pose(125, 25, Math.toRadians(135)); // Scoring/high basket position

    private final Pose preStaged1Pose = new Pose(115, 25, Math.toRadians(180)); // First Pre-staged sample pickup
    private final Pose preStaged2Pose = new Pose(115, 15, Math.toRadians(180)); // Second Pre-staged sample pickup
    private final Pose preStaged3Pose = new Pose(115, 12, Math.toRadians(215)); // Third Pre-staged sample pickup
    private final Pose subPose = new Pose(72, 40, Math.toRadians(90)); // sub pickup/position
    private final Pose endPose = new Pose(72, 45, Math.toRadians(-90));    // Endgame position - low rung

    private Path scorePreload, park;
    private PathChain grabPreStaged1, grabPreStaged2, grabPreStaged3, grabSub, scorePreStaged1, scorePreStaged2, scorePreStaged3, scoreSub;

    public void buildPaths() {
        // Path for scoring preload
        scorePreload = new Path(new BezierCurve(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

        // Path chains for picking up and scoring samples
        grabPreStaged1 = Follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose), new Point(preStaged1Pose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), preStaged1Pose.getHeading())
                .build();

        scorePreStaged1 = Follower.pathBuilder()
                .addPath(new BezierLine(new Point(preStaged1Pose), new Point(scorePose)))
                .setLinearHeadingInterpolation(preStaged1Pose.getHeading(), scorePose.getHeading())
                .build();

        grabPreStaged2 = Follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose), new Point(preStaged2Pose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), preStaged2Pose.getHeading())
                .build();

        scorePreStaged2 = Follower.pathBuilder()
                .addPath(new BezierLine(new Point(preStaged2Pose), new Point(scorePose)))
                .setLinearHeadingInterpolation(preStaged2Pose.getHeading(), scorePose.getHeading())
                .build();

        grabPreStaged3 = Follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose), new Point(preStaged3Pose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), preStaged3Pose.getHeading())
                .build();

        scorePreStaged3 = Follower.pathBuilder()
                .addPath(new BezierLine(new Point(preStaged3Pose), new Point(scorePose)))
                .setLinearHeadingInterpolation(preStaged3Pose.getHeading(), scorePose.getHeading())
                .build();

        grabSub = Follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose), new Point(subPose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), subPose.getHeading())
                .build();

        scoreSub = Follower.pathBuilder()
                .addPath(new BezierLine(new Point(subPose), new Point(scorePose)))
                .setLinearHeadingInterpolation(subPose.getHeading(), scorePose.getHeading())
                .build();

        // The .pp file does not give us the heading of the control pose for a curved path, so we will not be needing this code.
        // Curved path for parking
        // park = new Path(new BezierCurve(new Point(scorePose), new Point(parkControlPose), new Point(parkPose)));
        // park.setLinearHeadingInterpolation(scorePose.getHeading(), parkPose.getHeading());
    }
}

 */