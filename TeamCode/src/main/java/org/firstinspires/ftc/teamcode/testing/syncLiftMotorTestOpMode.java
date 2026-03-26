package org.firstinspires.ftc.teamcode.testing;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

@TeleOp(name = "Sync Elevoator Motor Test", group = "Testing")
public class syncLiftMotorTestOpMode extends LinearOpMode {
    public static double kF = 0;
    public static double kP = 0;
    public static double kI = 0;
    public static double kD = 0;

    static TelemetryManager panelsTelemetry;

    double ticksPerElevate = 1960.0/50.0;

    DcMotorEx liftMotor;

    @Override
    public void runOpMode() throws InterruptedException {
        liftMotor = hardwareMap.get(DcMotorEx.class, "elevatorMotor");
        ElapsedTime wait = new ElapsedTime();

        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        telemetry.setMsTransmissionInterval(100);

        // Need an Encoder

        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



        waitForStart();

        double power = 0;
        double encoder = 0;
        double velocity = 0;
        boolean running = false;
        boolean stopping = false;

        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        while (opModeIsActive()) {
            boolean aPressed= gamepad1.aWasPressed();
            boolean bPressed= gamepad1.bWasPressed();
            // liftMotor.setPower(power);
            if (aPressed && !running ) {
                liftMotor.setTargetPosition((int) (ticksPerElevate * 10)); // -1 or +1 or 0
                liftMotor.setPower(1.0);
                liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                running = true;
            }

            encoder = liftMotor.getCurrentPosition();
            if (Math.abs(encoder)>ticksPerElevate) // Wait for Indexer   (28ppr / (10T sprocket to 23<-1> TOOTH CHAIN)
            {
                // liftMotor.setPower(0.0); // chart 2 (346 ms)
                liftMotor.setTargetPosition((int) encoder); // chart 3
//                wait.reset();
//                while (wait.milliseconds() < 250) { } // wait 1/4 sec;
//                liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                running = false;
                stopping = true;
            }

            velocity = liftMotor.getVelocity();
            if (Math.abs(velocity) < 100 && stopping == true)
            {
                // stopped (ish)
                stopping = false;
                running = false;
                liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }

            if(bPressed) {
                setLiftMotorPIDFCoefficients();
            }

            telemetry.addData("Power", power);
            telemetry.addData("Encoder", encoder);

            telemetry.update();

            panelsTelemetry.addData("Power", power);
            panelsTelemetry.addData("Encoder", encoder);

            panelsTelemetry.update();

            RobotLog.d("Lifter pev: %.2f,%.2f,%.2f",power,encoder,velocity );
        }
    }

    //For launch motor coefficients testing only
    public void setLiftMotorPIDFCoefficients() {
        // Change coefficients using methods included with DcMotorEx class.
        PIDFCoefficients pidfNew = new PIDFCoefficients(kP, kI, kD, kF);
        liftMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfNew);
    }

}
