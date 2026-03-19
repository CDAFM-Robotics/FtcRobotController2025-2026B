package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.common.Robot;

@TeleOp(name = "KickStand Test", group = "Testing")
public class KickStandTestOpMode extends LinearOpMode {
    boolean isRedSide = false;
    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap, telemetry, isRedSide);
        // Servo leftKickStand;
        // Servo rightKickStand;
        // rightKickStand = hardwareMap.get(Servo.class, "rightKickStand");
        // leftKickStand = hardwareMap.get(Servo.class, "leftKickStand");

        telemetry.setMsTransmissionInterval(50);

        double led_power = 0.0;
        double position_left = 0.0;
        double position_right = 0.0;

        Gamepad currentGamepad1 = new Gamepad();
        Gamepad previousGamepad1 = new Gamepad();
        Gamepad currentGamepad2 = new Gamepad();
        Gamepad previousGamepad2 = new Gamepad();

        waitForStart();

        // 2025 - 2026 (Archimedes)
        // Control HUB
        // 0 Indexer Servo (Axoon)
        // 1 Kicker
        // 2 Undef
        // 3 UNdef
        // 4 kickStandLight
        // 5 rightKickStand

        // Exp Hub
        // 0 leftKickStand
        // 1 Undef
        // 2 Undef
        // 3 Undef
        // 4 Undef
        // 5 Undef

        while (opModeIsActive()) {

            previousGamepad1.copy(currentGamepad1);
            previousGamepad2.copy(currentGamepad2);
            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);

            if (currentGamepad1.a && !previousGamepad1.a) {
                position_left += 0.1;
            }
            if (currentGamepad1.b && !previousGamepad1.b) {
                position_left -= 0.1;
            }
            // leftKickStand.setPosition(position_left);

            if (currentGamepad1.right_bumper && !previousGamepad1.right_bumper) {
                position_right += 0.1;
            }
            if (currentGamepad1.left_bumper && !previousGamepad1.left_bumper) {
                position_right -= 0.1;
            }
            // rightKickStand.setPosition(position_right);





            // X/Y Kick Stand   (Left = ControlHub P5 / Right = ExpHub P0)
            // SQ TRI ps5 controller
            if (currentGamepad1.x != previousGamepad1.x) {
                robot.getDriveBase().setKickStand();
            }

            if (currentGamepad1.y != previousGamepad1.y) {
                robot.getDriveBase().resetKickStand();
            }


             // U/D Kick Stand Lights
            if (currentGamepad1.dpad_up != previousGamepad1.dpad_up) {
                // robot.getDriveBase().setKickStandLight();
                led_power = 1.0;
            }

            if (currentGamepad1.dpad_down != previousGamepad1.dpad_down) {
                // robot.getDriveBase().resetKickStandLight();
                led_power = 0.0;
            }

            // L/R Kick Stand Lights
            if (currentGamepad1.dpad_right != previousGamepad1.dpad_right) {
                led_power += 0.1;
                if (led_power >= 1.0)
                {
                    led_power = 1.0;
                }
                //robot.getDriveBase().adjustKickStandLight(led_power);
            }

            if (currentGamepad1.dpad_left != previousGamepad1.dpad_left) {
                led_power -= 0.1;
                if (led_power <= 0.0)
                {
                    led_power = 0.0;
                }
                //robot.getDriveBase().adjustKickStandLight(led_power);
            }

            telemetry.addLine("GP1 X/Y: KickStand in Robot,  Up/Dn: kickStandLight, a/b and r/lbumper kickstand incremental");
            telemetry.addData("light power: ", led_power);
            telemetry.addData("kickstand left ", position_left);
            telemetry.addData("kickstand right ", position_right);
            telemetry.update();
        }
    }
}
