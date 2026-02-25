package org.firstinspires.ftc.teamcode.pedropathing.commands;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public class Paths {
    private PathChain blueFarPickupFirstMark;
    private PathChain blueFarPickupSecondMark;
    private PathChain blueFarPickupThirdMark;
    private PathChain blueFarReturnFromThirdMark;
    private PathChain blueFarPickupHumanPlayerZone;

    Follower follower;

    public Paths(Follower follower) {
        this.follower = follower;
    }

    public Follower getFollower() {
        return follower;
    }

    /*

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

     */

    /*

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

     */

    public PathChain getBlueFarPickupThirdMark() {
        blueFarPickupThirdMark = follower.pathBuilder().addPath(
                new BezierCurve(
                    new Pose(56.000, 11.000),
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

                    new Pose(56.000, 11.000)
                )
            ).setLinearHeadingInterpolation(Math.toRadians(-176), Math.toRadians(90))

            .build();

        return blueFarReturnFromThirdMark;
    }

    /*

    public PathChain getBlueFarPickupHumanPlayerZone() {
        blueFarPickupHumanPlayerZone = follower.pathBuilder()
            .addPath(
                new BezierLine(
                    new Pose(56.000, 8.500),
                    new Pose(8.000, 8.500)
                )).setConstantHeadingInterpolation(Math.toRadians(180))
            .addPath(
                new BezierLine(
                    new Pose(8.000, 8.500),
                    new Pose(56.000, 8.500)
            )).setConstantHeadingInterpolation(Math.toRadians(180))

            .build();

        return blueFarPickupHumanPlayerZone;
    }

     */
}

