package org.firstinspires.ftc.teamcode.common;

public class RobotStaticValuesClass {

    public enum Oblisk {
        GPP,
        PGP,
        PPG
    }

    // Static variables persist between OpModes
    public static Pose2D savedPose = new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.RADIANS, 0);
    public static Oblisk savedOblisk = Oblisk.GPP;
    public static double turretAngleOffset = 0.0;
    public static boolean autoCompleted = false;

}
