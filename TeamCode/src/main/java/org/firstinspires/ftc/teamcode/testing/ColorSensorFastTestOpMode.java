package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.common.subsystems.ColorSensor;

// THis is the same as ColorSensorTestOpMode but relies on ColorSensor.kt module for fast reading
// performs 6 color sensor distance,color lookup and ball detection in ~15ms
@TeleOp(name = "Color Sensor FAST Test", group = "Testing")
// @Disabled
public class ColorSensorFastTestOpMode extends LinearOpMode {

    ColorSensor colorSensorIntakeLeft;
    ColorSensor colorSensorIntakeRight;

    ColorSensor colorSensorOutputFront;
    ColorSensor colorSensorOutputB;

    ColorSensor colorSensorAltFront;
    ColorSensor colorSensorAltB;


    NormalizedRGBA colorSensorANormalizedColors;
    NormalizedRGBA colorSensorBNormalizedColors;

    enum ArtifactColor {
        PURPLE,
        GREEN,
        NONE,
        UNKNOWN
    }

    ArtifactColor colorSensorADetectedColor = null;
    ArtifactColor colorSensorBDetectedColor = null;

    ArtifactColor predictedColor = null;


    double sensorADistance = 0.0;
    double sensorBDistance = 0.0;
    float gain = 8;


    @Override
    public void runOpMode() {


    /*    colorSensorIntakeLeft = hardwareMap.get(NormalizedColorSensor.class, "colorSensorIntakeLeftLeft");
        colorSensorIntakeRight = hardwareMap.get(NormalizedColorSensor.class, "colorSensorIntakeLeftRight");
        colorSensorOutputFront = hardwareMap.get(NormalizedColorSensor.class, "colorSensorOutLeft");
        colorSensorOutputB = hardwareMap.get(NormalizedColorSensor.class, "colorSensorOutRight");
        colorSensorAltFront = hardwareMap.get(NormalizedColorSensor.class, "colorSensorIntakeRightLeft");
        colorSensorAltB = hardwareMap.get(NormalizedColorSensor.class, "colorSensorIntakeRightRight");

     */
        // TODD test new faster
        colorSensorIntakeLeft = new ColorSensor(hardwareMap,"colorSensorIntakeLeftLeft");
        colorSensorIntakeLeft.initialize();
        colorSensorIntakeRight = new ColorSensor(hardwareMap,"colorSensorIntakeLeftRight");
        colorSensorIntakeRight.initialize();
        colorSensorOutputFront = new ColorSensor(hardwareMap,"colorSensorOutRight");
        colorSensorOutputFront.initialize();
        colorSensorOutputB = new ColorSensor(hardwareMap,"colorSensorOutLeft");
        colorSensorOutputB.initialize();
        colorSensorAltFront = new ColorSensor(hardwareMap, "colorSensorIntakeRightLeft");
        colorSensorAltFront.initialize();
        colorSensorAltB = new ColorSensor(hardwareMap, "colorSensorIntakeRightRight");
        colorSensorAltB.initialize();


        waitForStart();

        while (opModeIsActive()) {




            telemetry.addLine("Color Sensor Intake");
            getColorSensor(colorSensorIntakeLeft, colorSensorIntakeRight);
            telemetry.addLine();
            telemetry.addLine("Color Sensor Output");
            getColorSensor(colorSensorOutputFront, colorSensorOutputB);
            telemetry.addLine();
            telemetry.addLine("Color Sensor Alt");
            getColorSensor(colorSensorAltFront, colorSensorAltB);

            telemetry.update();


        }
    }

    public void getColorSensor(ColorSensor colorSensorA, ColorSensor colorSensorB)
    {
        // Update the color sensors
        RobotLog.d("CS A 0");
        colorSensorA.periodic();
        colorSensorB.periodic();

        //colorSensorA.setGain(gain);
        //colorSensorB.setGain(gain);

        RobotLog.d("CS A 1");
        colorSensorANormalizedColors = colorSensorA.getColors();
        RobotLog.d("CS A 2");
        sensorADistance = colorSensorA.getDistance();
        RobotLog.d("CS A 3");

        // TODO Changed "Detect Distance to 6.5 for both (Output and Alternate Sensor Distance in real program)
        // TODO: ~15 = Black Divider wall
        // TODO: ~15 = No Ball
        // TODO: BUT having detect distance too high may cause false-trigger on intake slot causing ball-stuck (sugg: ~3-4.5)

        if (sensorADistance > 2) {
            colorSensorADetectedColor = ArtifactColor.NONE;
        }
        else if (colorSensorANormalizedColors.blue > colorSensorANormalizedColors.green) {
            colorSensorADetectedColor = ArtifactColor.PURPLE;
        }
        else { // telemetry.addData("Predicted Color", ArtifactColor.UNKNOWN);
            colorSensorADetectedColor = ArtifactColor.GREEN;
        }

        colorSensorBNormalizedColors = colorSensorB.getColors();
        sensorBDistance = colorSensorB.getDistance();

        if (sensorBDistance > 2) {
            colorSensorBDetectedColor = ArtifactColor.NONE;
        }
        else if (colorSensorBNormalizedColors.blue > colorSensorBNormalizedColors.green) {
            colorSensorBDetectedColor = ArtifactColor.PURPLE;
        }
        else {
            colorSensorBDetectedColor = ArtifactColor.GREEN;
        }

        // Predicted Color
        if (colorSensorADetectedColor == colorSensorBDetectedColor) {
            predictedColor =  colorSensorADetectedColor;
        }
        else if (colorSensorBDetectedColor == ArtifactColor.NONE) {
            predictedColor = colorSensorADetectedColor;
        }
        else if (colorSensorADetectedColor == ArtifactColor.NONE){
            predictedColor = colorSensorBDetectedColor;
        }
        else {
            predictedColor = ArtifactColor.UNKNOWN;
        }

        telemetry.addLine()
                .addData("RGBA", "%.2f, %.2f, %.2f, %.2f / %.2f, %.2f, %.2f, %.2f",
                        colorSensorANormalizedColors.red, colorSensorANormalizedColors.green, colorSensorANormalizedColors.blue, colorSensorANormalizedColors.alpha,
                        colorSensorBNormalizedColors.red, colorSensorBNormalizedColors.green, colorSensorBNormalizedColors.blue, colorSensorBNormalizedColors.alpha);
        telemetry.addData("Distance (cm)", "%.2f / %.2f", sensorADistance, sensorBDistance);
        telemetry.addData("Detected Color",  "%s / %s", colorSensorADetectedColor, colorSensorBDetectedColor);
        telemetry.addData("Predicted Color", predictedColor);
        RobotLog.d("RGBA: %.2f, %.2f, %.2f, %.2f / %.2f, %.2f, %.2f, %.2f", colorSensorANormalizedColors.red, colorSensorANormalizedColors.green, colorSensorANormalizedColors.blue, colorSensorANormalizedColors.alpha,
            colorSensorBNormalizedColors.red, colorSensorBNormalizedColors.green, colorSensorBNormalizedColors.blue, colorSensorBNormalizedColors.alpha);
        RobotLog.d("DIST: %.2f / %.2f",sensorADistance, sensorBDistance );

    }
}
