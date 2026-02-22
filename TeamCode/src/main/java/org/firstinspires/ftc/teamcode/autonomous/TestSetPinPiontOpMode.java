package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.common.RobotStaticValuesClass;

@Autonomous(name = "Test Pinpoint OpMode", group = "Testing")
public class TestSetPinPiontOpMode extends LinearOpMode {
    GoBildaPinpointDriver pinpoint;

    @Override
    public void runOpMode() throws InterruptedException {

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        // TODO TESTING Orientation may be wrong.
        pinpoint.setOffsets(8, -3.25, DistanceUnit.INCH); //Tuned for 2026 Bot2 17Feb26 OK

        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
            GoBildaPinpointDriver.EncoderDirection.REVERSED);

        // Set starting position on field (e.g. blue alliance)
        pinpoint.resetPosAndIMU();
        sleep(500);
        telemetry.addData("Pinpoint2"," Heading %.2f degrees: %.2f, Pos x:%.2f y:%.2f", pinpoint.getHeading(AngleUnit.DEGREES), pinpoint.getHeading(AngleUnit.RADIANS), pinpoint.getPosX(DistanceUnit.INCH),pinpoint.getPosY(DistanceUnit.INCH));
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            pinpoint.update();
            // ... your auto code ...
        }

        // Save final pose at the very end of auto
        RobotStaticValuesClass.savedOblisk = RobotStaticValuesClass.Oblisk.PGP;
        RobotStaticValuesClass.savedPose = pinpoint.getPosition();
        RobotStaticValuesClass.autoCompleted = true;

    }
}