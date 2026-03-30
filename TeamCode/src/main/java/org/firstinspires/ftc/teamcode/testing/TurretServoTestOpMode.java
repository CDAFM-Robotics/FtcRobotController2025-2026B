package org.firstinspires.ftc.teamcode.testing;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.RobotLog;


@TeleOp(name = "Bot2 Turret Servo Test", group = "Testing")
@Configurable
public class TurretServoTestOpMode extends LinearOpMode {

    public static double kF = 0.07; // 0.09; // 0.12
    public static double kP = 0.008; // 0.002; // 0.01
    public static double kI = 0.0; // 0.0004;
    public static double kD = 0.0; // 0.0002; // 0.0

    public static double target = 0.0;
    public static double deadband = 0; // 1.5;

    static TelemetryManager panelsTelemetry;
    // TODO Panels telemetry


    @Override
    public void runOpMode() {


        CRServo launcherServo = hardwareMap.get(CRServo.class, "turretServo");
        AnalogInput launcherAnalogInput = hardwareMap.get(AnalogInput.class, "turretAnalog");

        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        telemetry.setMsTransmissionInterval(100);


        double currentVoltage;
        double currentAngle;
        double currentAngleOffset = 0;

        double lastVoltage = 0;
        double lastAngle = 0;

        double actualAngle;

        double power = 0;

        double diff;
        double greatestDiff = -0x80000000;

        boolean firstLoop = true;
        boolean pid = true;

        while (opModeInInit()) {
            currentVoltage = launcherAnalogInput.getVoltage();
            // currentAngle = currentVoltage / 3.3 * 360;
            currentAngle = ((currentVoltage / 3.3 * 360) - 2.5) / 352.5 * 360;
            // currentAngle = currentVoltage / 3.19 * 360; // TODO new one

            //348.55 is max with no power
            // 378.33 with power
            //369 with -power

            // min is 0
            // 38.84 with power
            // 25 with - power

            actualAngle = currentAngle + currentAngleOffset;

            telemetry.addData("Servo Voltage", "%.2f", currentVoltage);
            telemetry.addData("Servo Angle Raw", "%.2f", currentAngle);
            telemetry.addLine();
            telemetry.addData("Last Servo Voltage", "%.2f", lastVoltage);
            telemetry.addData("Last Servo Angle Raw", "%.2f", lastAngle);
            telemetry.addLine();
            telemetry.addData("Actual Servo Angle", "%.2f", actualAngle);

            telemetry.update();

            lastAngle = currentAngle;
            lastVoltage = currentVoltage;
        }

        waitForStart();

        Gamepad prevGamepad1 = new Gamepad();
        Gamepad currentGamepad1 = new Gamepad();

        while (opModeIsActive()) {

            // Find the voltage returned and the angle of the servo

            currentVoltage = launcherAnalogInput.getVoltage();

            // TODO: Voltage only outputs angles between 2.5 deg and 355 deg
            currentAngle = ((currentVoltage / 3.3 * 360) - 2.5) / 352.5 * 360; // max 355, min 2.5
            // currentAngle = currentVoltage / 3.3 * 360.0;
            // currentAngle = currentVoltage / 3.19 * 360;

            // Find out whether the angle looped around

            diff = Math.abs(currentAngle - lastAngle);

            if (!firstLoop) {
                if (currentAngle > 180 && lastAngle <= 180 && diff > 100) { // Diff tuned by jo on 19m26 with panels to prevent flip
                    currentAngleOffset -= 360;
                }

                if (currentAngle < 180 && lastAngle >= 180 && diff > 100) {
                    currentAngleOffset += 360;
                }
            }

            actualAngle = currentAngle + currentAngleOffset;

            greatestDiff = Math.max(greatestDiff, diff);

            // Set Servo power

            if (gamepad1.a) {
                pid = true;
                lastTime = System.nanoTime() / 1000000000.0;
                integralSum = 0;
            }
            if (gamepad1.b) {
                pid = false;
            }

            if (!pid) {
                power = gamepad1.left_stick_x;

                if (power == 0.0)
                {
                    power = 0.001; // TODO test set a little power to see if it affects getVoltage min/max
                }
                launcherServo.setPower(power);
            }
            else {
                power = updatePID(target, actualAngle);
                launcherServo.setPower(power);
            }

            // Add telemetry data for debugging

            telemetry.addData("Power", power);
            telemetry.addLine();
            telemetry.addData("Servo Voltage", "%.2f", currentVoltage);
            telemetry.addData("Servo Angle Raw", "%.2f", currentAngle);
            telemetry.addLine();
            telemetry.addData("Last Servo Voltage", "%.2f", lastVoltage);
            telemetry.addData("Last Servo Angle Raw", "%.2f", lastAngle);
            telemetry.addLine();
            telemetry.addData("Difference", "%.2f", diff);
            telemetry.addData("Angle Offset", "%.2f", currentAngleOffset);

            telemetry.addData("Actual Servo Angle", "%.2f", actualAngle);

            telemetry.update();

            // Logging

            RobotLog.d("Power: %.2f, Servo Angle: %.2f, Last Servo Angle: %.2f, Difference: %.2f, Angle Offset: %.2f, Actual Servo Angle: %.2f", power, currentAngle, lastAngle, diff, currentAngleOffset, actualAngle);

            // Set last variables for next loop

            lastAngle = currentAngle;
            lastVoltage = currentVoltage;

            firstLoop = false;

            prevGamepad1.copy(currentGamepad1);
            currentGamepad1.copy(gamepad1);

            // TODO put to panels
            panelsTelemetry.addData("Turret Angle", currentAngle);
            panelsTelemetry.addData("Actual Angle", actualAngle);
            panelsTelemetry.addData("Target", target);
            panelsTelemetry.update();
        }
    }

    private double integralSum = 0;
    private double lastError = 0;
    private double lastTime = 0;
    private double turretTime = 0;

    public double updatePID(double target, double current) {
        lastTime = turretTime;
        turretTime = System.nanoTime() / 1000000000.0;
        double dt = turretTime - lastTime;

        double error = target - current;
        double gain = 0.85;



        if ((error >= 0 && lastError < 0) || (error <= 0 && lastError > 0)) {
            integralSum = 0;
        }

        integralSum += error * dt;

        double derivative = (error - lastError) / dt;


        telemetry.addData("Error", "%.2f", error);
        telemetry.addData("Integral", "%.2f", integralSum);
        telemetry.addData("Derivative", "%.2f", derivative);
        telemetry.addLine();

        panelsTelemetry.addData("Error", error);
        panelsTelemetry.addData("Integral", integralSum);
        panelsTelemetry.addData("Derivative", derivative);

        lastError = error;

        if (Math.abs(error) < deadband) {
            return 0.001; // TODO add tiny bit of power to see if it changes getVoltage
        }

        return Math.max(Math.min((error * kP) + (integralSum * kI) + (derivative * kD) + (kF * Math.signum(error)), 1), -1);
    }
}
