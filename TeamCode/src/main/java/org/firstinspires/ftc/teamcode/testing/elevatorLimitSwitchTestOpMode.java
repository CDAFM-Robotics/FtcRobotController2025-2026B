package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.TouchSensor;

@TeleOp(name = "elevatorLimitSwitchTestOpMode", group = "0testing")
public class elevatorLimitSwitchTestOpMode extends LinearOpMode {

    DcMotor elevatorMotor;
    TouchSensor magneticLimitSwitch;
    @Override
    public void runOpMode() throws InterruptedException {
        elevatorMotor = hardwareMap.get(DcMotor.class, "backRightMotor");
        magneticLimitSwitch = hardwareMap.get(TouchSensor.class, "magneticLimitSwitch");

        elevatorMotor.setTargetPosition(0);
        elevatorMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Gamepad currentGamepad1 = new Gamepad();
        Gamepad previousGamepad1 = new Gamepad();
        Gamepad currentGamepad2 = new Gamepad();
        Gamepad previousGamepad2 = new Gamepad();

        waitForStart();
        while(opModeIsActive()){
            previousGamepad1.copy(currentGamepad1);
            previousGamepad2.copy(currentGamepad2);
            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);

            if(currentGamepad1.a && !previousGamepad1.a){
                elevatorMotor.setPower(1);
                elevatorMotor.setTargetPosition(elevatorMotor.getTargetPosition() + 50);
            }
            if(magneticLimitSwitch.isPressed()){
                elevatorMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                elevatorMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                elevatorMotor.setTargetPosition(0);
            }

            telemetry.addData("current position", elevatorMotor.getCurrentPosition());
            telemetry.addData("target position", elevatorMotor.getTargetPosition());
            telemetry.addData("limit switch detect", magneticLimitSwitch.isPressed());
            telemetry.update();
        }
    }
}
