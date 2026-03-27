package org.firstinspires.ftc.teamcode.common;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class RobotStaticValuesClass {

    public enum Oblisk {
        GPP,
        PGP,
        PPG,
        UNKNOW
    }

    // Static variables persist between OpModes
    public static boolean autoCompleted = false;
    public static boolean teleOpCompleted = false;
    public static Pose2D savedPose = new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.RADIANS, 0);
    public static Oblisk savedOblisk = Oblisk.UNKNOW; //default value
    public static double turretAngleOffset = 0.0;

    public static void saveState(Pose2D finalPose2D,
              double currentAngleOffset,
    RobotStaticValuesClass.Oblisk oblisk) {
        savedPose           = finalPose2D;
        turretAngleOffset   = currentAngleOffset;
        savedOblisk         = oblisk;
    }

}