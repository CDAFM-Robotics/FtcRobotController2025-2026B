package org.firstinspires.ftc.teamcode.pedropathing.commands;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public class Paths {
    private PathChain blueFarPickupFirstMark;
    private PathChain blueFarReturnFromFirstMark;
    private PathChain blueFarPickupSecondMark;
    private PathChain blueFarReturnFromSecondMark;
    private PathChain blueFarPickupThirdMark;
    private PathChain blueFarReturnFromThirdMark;
    private PathChain blueFarPickupHumanPlayerZone;
    private PathChain blueFarReturnFromHumanPlayerZone;

    private PathChain blueCloseStartToShoot;
    private PathChain blueCloseStartToShoot2; // TODO jw test
    private PathChain blueClosePickupFirstMark;
    private PathChain blueCloseReturnFromFirstMark;
    private PathChain blueClosePickupSecondMark;
    private PathChain blueCloseHitGate;
    private PathChain blueCloseReturnFromSecondMark;
    private PathChain blueClosePickupThirdMark;
    private PathChain blueCloseReturnFromThirdMark;

    Follower follower;

    public Paths(Follower follower) {
        this.follower = follower;
    }

    public Follower getFollower() {
        return follower;
    }

    public PathChain getBlueFarPickupFirstMark() {
        blueFarPickupFirstMark = follower.pathBuilder().addPath(
                new BezierCurve(
                    new Pose(56.000, 8.500),
                    new Pose(63.000, 109.000),
                    new Pose(28.000, 74.000),
                    new Pose(1.000, 96.000),
                    new Pose(15.000, 89.000),
                    new Pose(56.000, 8.500)
                )
            ).setConstantHeadingInterpolation(Math.toRadians(180))
            .build();

        return blueFarPickupFirstMark;
    }

    public PathChain getBlueFarReturnFromFirstMark() {
        return blueFarReturnFromFirstMark;
    }

    public PathChain getBlueFarPickupSecondMark() {
        blueFarPickupSecondMark = follower.pathBuilder().addPath(
                new BezierCurve(
                    new Pose(56.000, 8.500),
                    new Pose(43.000, 99.000),
                    new Pose(2.000, 59.000),
                    new Pose(2.000, 38.000),
                    new Pose(56.000, 8.500)
                )
            ).setConstantHeadingInterpolation(Math.toRadians(180))
            .build();

        return blueFarPickupSecondMark;
    }

    public PathChain getBlueFarReturnFromSecondMark() {
        return blueFarReturnFromSecondMark;
    }

    public PathChain getBlueFarPickupThirdMark() {
        blueFarPickupThirdMark = follower.pathBuilder().addPath(
                new BezierCurve(
                    new Pose(56.000, 8.5),
                    new Pose(57.314, 38.264),
                    new Pose(11.000, 36.000)
                )
            ).setTangentHeadingInterpolation()

            .build();

        return blueFarPickupThirdMark;
    }

    public PathChain getBlueFarReturnFromThirdMark() {
        blueFarReturnFromThirdMark = follower.pathBuilder().addPath(
                new BezierLine(
                    new Pose(11.000, 36.000),

                    new Pose(56.000, 14)
                )
            ).setLinearHeadingInterpolation(Math.toRadians(-176), Math.toRadians(180))

            .build();

        return blueFarReturnFromThirdMark;
    }

    public PathChain getBlueFarPickupHumanPlayerZone() {
        blueFarPickupHumanPlayerZone = follower.pathBuilder()
            .addPath(
                new BezierLine(
                    new Pose(56.000, 14),
                    new Pose(8.000, 10)
                )).setConstantHeadingInterpolation(Math.toRadians(180))

            .build();

        return blueFarPickupHumanPlayerZone;
    }
    public PathChain getBlueFarReturnFromHumanPlayerZone() {
        blueFarReturnFromHumanPlayerZone = follower.pathBuilder()
            .addPath(
                new BezierLine(
                    new Pose(8.000, 10),
                    new Pose(56.000, 14)
                )).setConstantHeadingInterpolation(Math.toRadians(180))

            .build();

        return blueFarReturnFromHumanPlayerZone;
    }

    public PathChain getBlueCloseStartToShoot() {
        blueCloseStartToShoot = follower.pathBuilder().addPath(
                new BezierLine(
                    new Pose(18.194, 121.659),
                    new Pose(60.000, 84.000)
                )
            ).setLinearHeadingInterpolation(Math.toRadians(143.5 - 180), Math.toRadians(180)).setReversed()

            .build();

        return blueCloseStartToShoot;
    }



//    public PathChain getBlueCloseStartToShoot2() {
//        blueCloseStartToShoot2 = follower.pathBuilder().addPath(
//                        new BezierLine(
//                                new Pose(17.766, 119.169),
//
//                                new Pose(53.519, 100.325)
//                        )
//                ).setLinearHeadingInterpolation(Math.toRadians(48), Math.toRadians(-36.51231))
//                .setReversed()
//                .build();
//        return blueCloseStartToShoot2;
//    }



    public PathChain getBlueCloseStartToShoot2() {
        blueCloseStartToShoot2 = follower.pathBuilder().addPath(
            new BezierLine(
                new Pose(17.766, 120.935),

                new Pose(54.000, 84.000)
            )
            ).setLinearHeadingInterpolation(Math.toRadians(53.5), Math.toRadians(90))
            .build();
        return blueCloseStartToShoot2;
    }





    public PathChain getBlueClosePickupFirstMark() {
        blueClosePickupFirstMark = follower.pathBuilder().addPath(
                new BezierLine(
                    new Pose(60.000, 84.000),

                    new Pose(15.000, 84.000) // 21
                )
            ).setConstantHeadingInterpolation(Math.toRadians(180))

            .build();

        return blueClosePickupFirstMark;
    }

    public PathChain getBlueCloseReturnFromFirstMark() {
        blueCloseReturnFromFirstMark = follower.pathBuilder().addPath(
                new BezierLine(
                    new Pose(15.000, 84.000),

                    new Pose(60.000, 84.000)
                )
            ).setConstantHeadingInterpolation(Math.toRadians(180))

            .build();

        return blueCloseReturnFromFirstMark;
    }

    public PathChain getBlueClosePickupSecondMark() {
        blueClosePickupSecondMark = follower.pathBuilder().addPath(
                new BezierCurve(
                    new Pose(60.000, 84.000),
                    new Pose(66.000, 54.000),
                    new Pose(17, 57.5) // x:21
                )
            ).setConstantHeadingInterpolation(Math.toRadians(180))

            .build();

        return blueClosePickupSecondMark;
    }

    public PathChain getBlueCloseHitGate() {
        blueCloseHitGate = follower.pathBuilder().addPath(
                new BezierCurve(
                    new Pose(21.000, 60.000),
                    new Pose(42.000, 66.000),
                    new Pose(15.000, 72.000)
                )
            ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(90))

            .build();

        return blueCloseHitGate;
    }

    public PathChain getBlueCloseReturnFromSecondMark() {
        blueCloseReturnFromSecondMark = follower.pathBuilder().addPath(
                new BezierLine(
                    new Pose(17, 57.5), // x:21, y=58 temp change no gate
                    new Pose(60.000, 84.000)
                )
            ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))

            .build();

        return blueCloseReturnFromSecondMark;
    }

    public PathChain getBlueClosePickupThirdMark() {
        blueClosePickupThirdMark = follower.pathBuilder().addPath(
                new BezierCurve(
                    new Pose(60.000, 84.000),
                    new Pose(66.000, 30.000),
                    new Pose(16.00, 36.000) // x:21
                )
            ).setConstantHeadingInterpolation(Math.toRadians(180))

            .build();

        return blueClosePickupThirdMark;
    }

    public PathChain getBlueCloseReturnFromThirdMark() {
        blueCloseReturnFromThirdMark = follower.pathBuilder().addPath(
                new BezierLine(
                    new Pose(16.000, 36.000), //x:21
                    new Pose(60.000, 84.000)
                )
            ).setConstantHeadingInterpolation(Math.toRadians(180))

            .build();

        return blueCloseReturnFromThirdMark;
    }
}

