package org.firstinspires.ftc.teamcode.teleop;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.common.Robot;
import org.firstinspires.ftc.teamcode.common.RobotStaticValuesClass;

@TeleOp(name = "RSD", group = "0teleop")
public class RSD extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        while (opModeInInit())
        {
            telemetry.addLine("Press START to RESET Static Values");
            telemetry.addLine("");
            telemetry.addData("auto", "%s", RobotStaticValuesClass.autoCompleted);
            telemetry.addData("tele", "%s", RobotStaticValuesClass.teleOpCompleted);
            telemetry.addData("p", "%s", RobotStaticValuesClass.savedPose);
            telemetry.addData("turretAngleOffset", "%.2f", RobotStaticValuesClass.turretAngleOffset);
            telemetry.addData("obelisk", "%s", RobotStaticValuesClass.savedOblisk);

            telemetry.update();

        }
        while (opModeIsActive())
        {
            telemetry.addLine("RESET COMPLETE!: Use Far Position");
            telemetry.addLine("");
            RobotStaticValuesClass.autoCompleted=false;
            RobotStaticValuesClass.teleOpCompleted=false;
            RobotStaticValuesClass.savedPose = new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.RADIANS, 0);
            RobotStaticValuesClass.savedOblisk = RobotStaticValuesClass.Oblisk.UNKNOW;
            RobotStaticValuesClass.turretAngleOffset = 0.0;

            telemetry.addData("auto", "%s", RobotStaticValuesClass.autoCompleted);
            telemetry.addData("tele", "%s", RobotStaticValuesClass.teleOpCompleted);
            telemetry.addData("p", "%s", RobotStaticValuesClass.savedPose);
            telemetry.addData("turretAngleOffset", "$.2f", RobotStaticValuesClass.turretAngleOffset);
            telemetry.addData("obelisk", "%s", RobotStaticValuesClass.savedOblisk);

            telemetry.update();
        }

    }

    }
