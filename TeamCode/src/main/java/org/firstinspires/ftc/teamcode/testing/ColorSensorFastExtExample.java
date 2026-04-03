package org.firstinspires.ftc.teamcode.testing;


import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.common.subsystems.ColorSensor;

//import org.firstinspires.ftc.teamcode.subsystems.Intake;
//import org.firstinspires.ftc.teamcode.subsystems.Spindexer;
//import org.firstinspires.ftc.teamcode.util.TelemetryImplUpstreamSubmission;

@Configurable
@TeleOp(name = "Test: Color Sensor", group = "Test")
public class ColorSensorFastExtExample extends LinearOpMode {
    static TelemetryManager telemetryM;

    public static boolean on = true;

    @Override
    public void runOpMode() {
        // Initialize the color sensor subsystem
        ColorSensor colorSensor = new ColorSensor(hardwareMap, "colorSensor");
        colorSensor.initialize();

        // Set up telemetry to combine FTC telemetry with upstream submission
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        // Wait for the start command
        waitForStart();

        // Main loop: runs while the OpMode is active
        while (opModeIsActive()) {
            // Update sensor values
            if (on) {
                colorSensor.periodic();
            }


            // Publish sensor data to telemetry
            telemetry.addData("Distance", "%05.2fcm", colorSensor.getDistance());
            telemetry.addData("Hue", "%07.4f", colorSensor.getHsv().getH());
            telemetry.addData("Saturation", "%07.4f", colorSensor.getHsv().getS());
            telemetry.addData("Value", "%07.4f", colorSensor.getHsv().getV());
            telemetry.update();
            String result = (colorSensor.getDetectedArtifact() != null && colorSensor.getDetectedArtifact().name() != null)
                ? colorSensor.getDetectedArtifact().name()
                : "NONE";
            RobotLog.d("Distance: %05.2fcm detectedArtifact: %s",colorSensor.getDistance(), result);
            telemetryM.addData("Distance",colorSensor.getDistance());
            telemetryM.addData("DetectedColor", result);
            telemetryM.update();
            // addData("Color Sensor Detected Artifact", detectedArtifact?.name ?: "None")
        }
    }
}
