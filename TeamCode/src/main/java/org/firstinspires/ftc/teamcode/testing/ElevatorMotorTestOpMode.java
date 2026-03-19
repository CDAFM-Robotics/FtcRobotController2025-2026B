package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;

@TeleOp(name = "Elevator Motor Test", group = "Testing")
public class ElevatorMotorTestOpMode extends OpMode {

    // -2588
    // -628

    DcMotorEx elevatorMotor;

    Gamepad currentGamepad1 = new Gamepad();
    Gamepad currentGamepad2 = new Gamepad();
    Gamepad previousGamepad1 = new Gamepad();
    Gamepad previousGamepad2 = new Gamepad();

    DcMotorEx launcherMotor1;
    DcMotorEx launcherMotor2;

    @Override
    public void init() {

        launcherMotor1 = hardwareMap.get(DcMotorEx.class, "launcherMotor1");
        launcherMotor2 = hardwareMap.get(DcMotorEx.class, "launcherMotor2");

        launcherMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        launcherMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        launcherMotor1.setDirection(DcMotorSimple.Direction.REVERSE);
        launcherMotor2.setDirection(DcMotorSimple.Direction.FORWARD);
        launcherMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcherMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        elevatorMotor = hardwareMap.get(DcMotorEx.class, "elevatorMotor");

        elevatorMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        elevatorMotor.setTargetPosition(0);
        elevatorMotor.setPower(1);

        elevatorMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        elevatorMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        elevatorMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    double ticksPerElevate = 78.4;

    double target = 0;
    boolean running = false;

    boolean launcherSpin = false;

    @Override
    public void loop() {
        previousGamepad1.copy(currentGamepad1);
        previousGamepad2.copy(currentGamepad2);

        currentGamepad1.copy(gamepad1);
        currentGamepad2.copy(gamepad2);

        if (currentGamepad1.a && !previousGamepad1.a) {
            target += ticksPerElevate;
            running = true;
            elevatorMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        }
        if (running) {
            elevatorMotor.setPower(1);
            if (elevatorMotor.getCurrentPosition() >= target) {
                running = false;
                elevatorMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
        }
        else {
            elevatorMotor.setTargetPosition((int) Math.round(target));
        }

        if (currentGamepad1.b && !previousGamepad1.b) {
            if (launcherSpin) {
                launcherMotor1.setVelocity(0);
                launcherMotor2.setVelocity(0);
                launcherSpin = false;
            }
            else {
                launcherMotor1.setVelocity(1200);
                launcherMotor2.setVelocity(1200);
                launcherSpin = true;
            }
        }


        telemetry.addData("Target", "%.2f", target);
        telemetry.addData("Position", elevatorMotor.getCurrentPosition());
        telemetry.update();
    }
}
