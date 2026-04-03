package org.firstinspires.ftc.teamcode.pedropathing.commands;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public class Paths {
    private PathChain blueFarPickupFirstMark;
    private PathChain redFarPickupFirstMark; // TODO not done
    private PathChain blueFarReturnFromFirstMark; // TODO not done
    private PathChain blueFarPickupSecondMark; // TODO not DONE
    private PathChain blueFarReturnFromSecondMark; // TODO not DONE
    private PathChain blueFarPickupThirdMark;
    private PathChain redFarPickupThirdMark;
    private PathChain blueFarReturnFromThirdMark;
    private PathChain redFarReturnFromThirdMark;
    private PathChain blueFarPickupHumanPlayerZone;
    private PathChain redFarPickupHumanPlayerZone;
    private PathChain blueFarReturnFromHumanPlayerZone;
    private PathChain redFarReturnFromHumanPlayerZone;

    private PathChain blueCloseStartToShoot;
    private PathChain blueCloseStartToShoot2;
    private PathChain redCloseStartToShoot2;
    private PathChain blueClosePickupFirstMark;
    private PathChain blueCloseReturnFromFirstMark;
    private PathChain redClosePickupFirstMark;
    private PathChain redCloseReturnFromFirstMark;
    private PathChain blueClosePickupSecondMark;
    private PathChain redClosePickupSecondMark;
    private PathChain blueCloseHitGate;
    private PathChain blueCloseReturnFromSecondMark;
    private PathChain redCloseReturnFromSecondMark;
    private PathChain blueClosePickupThirdMark;
    private PathChain blueCloseReturnFromThirdMark;
    private PathChain redClosePickupThirdMark;
    private PathChain redCloseReturnFromThirdMark;




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

    // OK 27FEB
    public PathChain getBlueFarPickupThirdMark() {
        blueFarPickupThirdMark = follower.pathBuilder().addPath(
                new BezierCurve(
                    new Pose(60.000, 8.5),
                    new Pose(60.000, 37.0),
                    new Pose(10, 36.000)
                )
            ).setTangentHeadingInterpolation()

            .build();

        return blueFarPickupThirdMark;
    }

    // OK 27FEB
    public PathChain getBlueFarReturnFromThirdMark() {
        blueFarReturnFromThirdMark = follower.pathBuilder().addPath(
                new BezierLine(
                    new Pose(10, 36.000),

                    new Pose(60.000, 14)
                )
            ).setLinearHeadingInterpolation(Math.toRadians(-176), Math.toRadians(180))

            .build();

        return blueFarReturnFromThirdMark;
    }

    // OK 27FEB
    public PathChain getRedFarPickupThirdMark() {
        redFarPickupThirdMark = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(84.000, 8.5),
                                new Pose(84.000, 37.7),
                                new Pose(134, 36.000)
                        )
                ).setTangentHeadingInterpolation()

                .build();

        return redFarPickupThirdMark;
    }
    // OK 27FEB
    public PathChain getRedFarReturnFromThirdMark() {
        redFarReturnFromThirdMark = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(134, 36.000),

                                new Pose(84.000, 14)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(-2), Math.toRadians(0))

                .build();

        return redFarReturnFromThirdMark;
    }

    // OK 27FEB
    public PathChain getBlueFarPickupHumanPlayerZone() {
        blueFarPickupHumanPlayerZone = follower.pathBuilder()
            .addPath(
                new BezierLine(
                    new Pose(60.000, 14),
                    new Pose(9.5, 12)
                )).setConstantHeadingInterpolation(Math.toRadians(180))

            .build();

        return blueFarPickupHumanPlayerZone;
    }

    // OK 27FEB
    public PathChain getBlueFarReturnFromHumanPlayerZone() {
        blueFarReturnFromHumanPlayerZone = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(9.5, 12),
                                new Pose(60.000, 14)
                        )).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();

        return blueFarReturnFromHumanPlayerZone;
    }

    // OK 27FEB
    public PathChain getRedFarPickupHumanPlayerZone() {
        redFarPickupHumanPlayerZone = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(84.000, 14),
                                new Pose(136, 10)
                        )).setConstantHeadingInterpolation(Math.toRadians(0))

                .build();

        return redFarPickupHumanPlayerZone;
    }




    // OK 27FEB
    public PathChain getRedFarReturnFromHumanPlayerZone() {
        redFarReturnFromHumanPlayerZone = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(136.000, 10),
                                new Pose(84.000, 14)
                        )).setConstantHeadingInterpolation(Math.toRadians(0))

                .build();

        return redFarReturnFromHumanPlayerZone;
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



    // OK 27FEB
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

    // OK 27FEB
    public PathChain getRedCloseStartToShoot2() {
        redCloseStartToShoot2 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(125.4, 120.8),

                                new Pose(90.0, 84.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(126), Math.toRadians(90))
                .build();
        return redCloseStartToShoot2;
    }

    // TODO upd 27FEB some issue in vis, if error maybe use LinearHeading 90->0
    public PathChain getRedClosePickupSecondMark() {
        redClosePickupSecondMark = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(90.0, 84.000),
                                new Pose(78.000, 54.000),
                                new Pose(128, 57.5)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(0))

                .build();

        return redClosePickupSecondMark;
    }

    //OK 27FEB
    public PathChain getRedCloseReturnFromSecondMark() {
        redCloseReturnFromSecondMark = follower.pathBuilder().addPath(
                new BezierCurve(
                    new Pose(128.000, 57.500),
                    new Pose(99.572, 66.008),
                    new Pose(90.000, 84.000)
                )
            ).setConstantHeadingInterpolation(Math.toRadians(0))
            .build();

        return redCloseReturnFromSecondMark;
    }



    public PathChain getBlueClosePickupFirstMark() {
        blueClosePickupFirstMark = follower.pathBuilder().addPath(
                new BezierLine(
                    new Pose(60.000, 84.000),

                    new Pose(18.000, 84.000) // 21
                )
            ).setConstantHeadingInterpolation(Math.toRadians(180))

            .build();

        return blueClosePickupFirstMark;
    }

    // OK 27FEB
    public PathChain getRedClosePickupFirstMark() {
        redClosePickupFirstMark = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(84.000, 84.000),

                                new Pose(124.000, 84.000)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(0))

                .build();

        return redClosePickupFirstMark;
    }

    // OK 27FEB
    public PathChain getRedCloseReturnFromFirstMark() {
        redCloseReturnFromFirstMark = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(124.000, 84.000),

                                new Pose(84.000, 84.000)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(0))

                .build();

        return redCloseReturnFromFirstMark;
    }

    public PathChain getBlueCloseReturnFromFirstMark() {
        blueCloseReturnFromFirstMark = follower.pathBuilder().addPath(
                new BezierLine(
                    new Pose(18, 84.000),

                    new Pose(60.000, 84.000)
                )
            ).setConstantHeadingInterpolation(Math.toRadians(180))

            .build();

        return blueCloseReturnFromFirstMark;
    }



    // TODO upd 27FEB. viz seems weird. May try Linear from 90->180 if it breaks
    public PathChain getBlueClosePickupSecondMark() {
        blueClosePickupSecondMark = follower.pathBuilder().addPath(
                new BezierCurve(
                    new Pose(54.000, 84.000),
                    new Pose(66.000, 54.000),
                    new Pose(16, 57.5)
                )
            ).setConstantHeadingInterpolation(Math.toRadians(180))

            .build();

        return blueClosePickupSecondMark;
    }

    // OK 27FEB
    public PathChain getBlueCloseReturnFromSecondMark() {
        blueCloseReturnFromSecondMark = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(16, 57.5),
                                new Pose(54.000, 84.000)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        return blueCloseReturnFromSecondMark;
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






    // OK 27FEB
    public PathChain getBlueClosePickupThirdMark() {
        blueClosePickupThirdMark = follower.pathBuilder().addPath(
                new BezierCurve(
                    new Pose(54.000, 84.000),
                    new Pose(66.000, 30.000),
                    new Pose(15, 36.000) // x:21
                )
            ).setConstantHeadingInterpolation(Math.toRadians(180))
            .build();

        return blueClosePickupThirdMark;
    }

    // OK 27FEB
    public PathChain getBlueCloseReturnFromThirdMark() {
        blueCloseReturnFromThirdMark = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(15, 36.000), //x:21
                                new Pose(54.000, 84.000)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();

        return blueCloseReturnFromThirdMark;
    }

    // OK 27FEB
    public PathChain getRedClosePickupThirdMark() {
        redClosePickupThirdMark = follower.pathBuilder().addPath(
                new BezierCurve(
                    new Pose(90.000, 84.000),
                    new Pose(78.000, 30.000),
                    new Pose(128.000, 33.500)
                )
            ).setConstantHeadingInterpolation(Math.toRadians(0))

            .build();;

        return redClosePickupThirdMark;
    }

    // OK 27FEB
    public PathChain getRedCloseReturnFromThirdMark() {
        redCloseReturnFromThirdMark = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(128.000, 33.5),
                                new Pose(90.000, 84.000)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(0))

                .build();

        return redCloseReturnFromThirdMark;
    }
}

