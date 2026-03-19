package org.firstinspires.ftc.teamcode.teleop;

import static org.firstinspires.ftc.teamcode.pedropathing.Tuning.follower;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.internal.camera.delegating.DelegatingCaptureSequence;
import org.firstinspires.ftc.teamcode.common.Robot;
import org.firstinspires.ftc.teamcode.common.RobotStaticValuesClass;
import org.firstinspires.ftc.teamcode.common.subsystems.Hud;
import org.firstinspires.ftc.teamcode.common.subsystems.Launcher;
import org.firstinspires.ftc.teamcode.common.util.DebugManager;

import com.bylazar.configurables.PanelsConfigurables;

@Configurable
@TeleOp(name = "RED Bot2 ", group = "0teleop")
public class DriverControlWithIndexerRedTeleOp extends LinearOpMode {
    public boolean isRedSide = true;

    // Make a local HUD
    private Hud hud;


    // TODO add Data to Panels
    // static TelemetryManager telemetryM;


    @Override
    public void runOpMode() throws InterruptedException {

        // TODO Add Data to Dashboard Start
        // FtcDashboard dashboard = FtcDashboard.getInstance();
        // telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        // TODO Panels telemetry
        // telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        // One line to set up — pass your telemetry and a tag name
        DebugManager debugManager = new DebugManager(telemetry, "TELEOP");
        // ── Toggle these for competition vs. development ────────────
        debugManager.LOG_DRIVEBASE  = true;
        debugManager.LOG_PINPOINT   = true;
        debugManager.LOG_VISION     = true;
        debugManager.LOG_LAUNCHER   = true;
        debugManager.LOG_SPINDEXER  = true;
        debugManager.LOG_INTAKE     = true;
        debugManager.LOG_HUD        = true;
        // ───────────────────────────────────────────────────────────
        debugManager.addData("Red side", "%s", isRedSide);

        Robot robot = new Robot(hardwareMap, telemetry, isRedSide);

        double driveSpeed = 1;
        boolean fieldCentric = true;
        boolean waitForReverseTimer = false;
        int REVERSE_INTAKE_TIME = 500;
        boolean isIntaking = false;

        Gamepad currentGamepad1 = new Gamepad();
        Gamepad previousGamepad1 = new Gamepad();
        Gamepad currentGamepad2 = new Gamepad();
        Gamepad previousGamepad2 = new Gamepad();

        ElapsedTime rumbleLauncherTimer  = new ElapsedTime();
        rumbleLauncherTimer.reset();
        ElapsedTime reverseIntakeTimer  = new ElapsedTime();
        reverseIntakeTimer.reset();
        //robot.getLauncher().setLimelightPipeline(isRedSide);

        hud = new Hud(hardwareMap, telemetry);

        debugManager.update();

        waitForStart();

        while (opModeIsActive()){
            previousGamepad1.copy(currentGamepad1);
            previousGamepad2.copy(currentGamepad2);
            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);

            // Driving controls for the robot
            if (currentGamepad1.left_stick_button && !previousGamepad1.left_stick_button){
                driveSpeed = driveSpeed == 1 ? 0.5 : 1;
            }

            if (currentGamepad1.back && !previousGamepad1.back){
                fieldCentric = !fieldCentric;
            }

            // Disabled the driver's ability to reset robot heading
            // since we are keeping the heading from autonomous
//            if (currentGamepad1.start && !previousGamepad1.start){
//                robot.getDriveBase().resetIMU();
//                gamepad1.rumble(300);
//            }

            if (currentGamepad1.right_bumper != previousGamepad1.right_bumper) {
                driveSpeed = driveSpeed == 1 ? 0.5 : 1;
            }

            robot.getDriveBase().setMotorPowers(gamepad1.left_stick_x, -gamepad1.left_stick_y, -gamepad1.right_stick_x, driveSpeed, fieldCentric);

            // Kickstand control
            if (currentGamepad1.a != previousGamepad1.a) {
                robot.getDriveBase().setKickStand();
                //robot.getDriveBase().setKickStandLight();
            }

            if (currentGamepad1.b != previousGamepad1.b) {
                robot.getDriveBase().resetKickStand();
                //robot.getDriveBase().resetKickStandLight();
            }

            // Intake Balls. Add isSafeToStop()
            if (currentGamepad1.right_trigger != 0.0
                && robot.isSafeToStopOuttake()) {
                //telemetry.addLine("gameped 1 right trigger");
                // Robot entering intake state
                if (robot.getRobotInOutState() != Robot.RobotInOutStates.INTAKE) {
                    robot.setRobotState(Robot.RobotInOutStates.INTAKE);
                    //reset intake state
                    robot.setAutoIntakeState(Robot.AutoIntakeStates.INIT);
                    //start the intake rolling
                    robot.getIntake().startIntake();
                }
                //turn the indexer for intake
                robot.intakeWithIndexerTurn();
            }

            if (currentGamepad1.right_trigger == 0.0 && previousGamepad1.right_trigger != 0){
                //robot update artifact colors
                robot.getIntake().stopIntake();
                //TODO: reverse intake for 500 milliseconds if there are three ball already
                robot.setRobotState(Robot.RobotInOutStates.IDLE);
            }

            if (currentGamepad1.left_trigger != 0) {
                robot.getIntake().reverseIntake();
            }
            else if (currentGamepad1.left_trigger == 0.0 && previousGamepad1.left_trigger != 0){
                //robot update artifact colors
                robot.getIntake().stopIntake();
            }

            // TODO: When indexer stuck or out of alignment, recover the color of the balls

            // Launcher
            if (currentGamepad2.x && !previousGamepad2.x) {
                robot.getLauncher().toggleLauncher();
                if (robot.getLauncher().isLauncherActive()){
                    gamepad2.rumble(0.0,1.0,500);
                }
                else{
                    gamepad2.rumble(1.0,0.0,250);
                }
            }

            if (robot.getLauncher().isLauncherActive() && rumbleLauncherTimer.milliseconds() > 1000){
                gamepad2.rumble(0.0,1.0,500);
                rumbleLauncherTimer.reset();
            }

            if (currentGamepad2.dpad_up && !previousGamepad2.dpad_up) {
                robot.getLauncher().changeLauncherVelocity(20);
            }

            if (currentGamepad2.dpad_down && !previousGamepad2.dpad_down) {
                robot.getLauncher().changeLauncherVelocity(-20);
            }

            if (currentGamepad2.dpad_left && !previousGamepad2.dpad_left) {
                robot.getLauncher().changeHood(-0.01);
            }

            if (currentGamepad2.dpad_right && !previousGamepad2.dpad_right) {
                robot.getLauncher().changeHood(0.01);
            }

            //telemetry.addData("hoodServo postion", robot.getLauncher().getHoodServoPosition());
            //telemetry.addData("kickerServo postion", robot.getLauncher().getKickerServoPosition());
            //telemetry.addData("isLauncher active", robot.getLauncher().isLauncherActive());

            //Launch all balls in the robot.
            if (currentGamepad2.right_trigger != 0 && previousGamepad2.right_trigger == 0) {
                //set the shooting order
                robot.shootOrderNone();
            }

            // Shoot balls in motif pattern
            if (currentGamepad2.right_bumper && !previousGamepad2.right_bumper) {
                //set motif shooting order
                robot.shootOrderMotif();
            }

            // Shoot balls in motif pattern of by 1
            if (currentGamepad2.left_bumper && !previousGamepad2.left_bumper) {
                //set motif shooting order of by 1
                robot.shootOrderMotifOneOff();
            }

            // Shoot balls in motif pattern of by 2
            if (currentGamepad2.left_trigger != 0 && previousGamepad2.left_trigger == 0) {
                //set the shooting order of by 2
                robot.shootOrderMotifTwoOff();
            }

            if ((currentGamepad2.right_trigger != 0
                || currentGamepad2.left_trigger != 0
                || currentGamepad2.right_bumper
                || currentGamepad2.left_bumper)
                && robot.getRobotInOutState() != Robot.RobotInOutStates.INTAKE
                && robot.getRobotInOutState() != Robot.RobotInOutStates.OUTTAKE) {
                    robot.setRobotState(Robot.RobotInOutStates.OUTTAKE);
            }

            if ((currentGamepad2.right_trigger != 0
                || currentGamepad2.left_trigger != 0
                || currentGamepad2.right_bumper
                || currentGamepad2.left_bumper)
                && robot.getRobotInOutState() == Robot.RobotInOutStates.OUTTAKE) {
                robot.shootAllBalls();
            }

            if ((currentGamepad2.right_trigger == 0
                || currentGamepad2.left_trigger == 0
                || !currentGamepad2.right_bumper
                || !currentGamepad2.left_bumper)
                && !robot.isSafeToStopOuttake()) {
                robot.shootAllBalls();
            }

            if ((currentGamepad2.right_trigger == 0
                || currentGamepad2.left_trigger == 0
                || !currentGamepad2.right_bumper
                || !currentGamepad2.left_bumper)
                && robot.isSafeToStopOuttake()) {
                if (robot.getRobotInOutState() == Robot.RobotInOutStates.OUTTAKE) {
                    robot.setRobotState(Robot.RobotInOutStates.IDLE);
                }
            }

            //TODO: if auton did not savve oblisk, read limelight until find oblisk aprilTag

                //TODO: driver 1 would like the gamepad 1 to rumble when the robot pick up a ball
/*            if (robot.isIntake1Ball()) {
                gamepad1.rumble(250);
                robot.setIntak1BallOff();
            }

            if (robot.isIntake3Balls()) {
                gamepad1.rumble(500);
                robot.setIntak3BallsOff();
            }*/

            //change gamepad 2 light bar when sped up all the way
            /*if(robot.getLauncher().getLauncherVelocity() == robot.getLauncher().getLauncherTargetVelocity() && robot.getLauncher().getLauncherTargetVelocity() != 0.0){
                gamepad2.setLedColor(255, 255, 0, 20);
            }*/

            debugManager.addData("Blue TeleOp color s0:", "%s", robot.getIndexer().artifactColorArray[0]);
            debugManager.addData("Blue TeleOp color s1:", "%s", robot.getIndexer().artifactColorArray[1]);
            debugManager.addData("Blue TeleOp color s2:", "%s", robot.getIndexer().artifactColorArray[2]);

            // TODO Measure Loop time and launcher velocity
            // RobotLog.d("launcher velocity: %f",
            //        robot.getLauncher().getLauncherVelocity());

            // TODO spit it out to Panels graph
            // telemetryM.addData("Velocity", robot.getLauncher().getLauncherVelocity());
            // telemetryM.update(telemetry);

            // Refresh the indicator lights
            hud.setBalls(robot.getIndexer().artifactColorArray[0], robot.getIndexer().artifactColorArray[1],robot.getIndexer().artifactColorArray[2]);
//            if (llLastIsValid == true)
//            {
//                // RobotLog.d("Aim PID X: %f", xAngle);
//                if (xAngle < Launcher.aimErrorTolerance)
//                {
//                    robot.getHud().setAimIndicator(true);
//                }
//            }
//            else {
//                robot.getHud().setAimIndicator(false);
//            }
            hud.UpdateBallUI();

            // TODO Add timing Log at end of loop
            debugManager.log("Blue TeleOp c0: %s c1: %s c2: %s",
                    robot.getIndexer().artifactColorArray[0],
                    robot.getIndexer().artifactColorArray[1],
                    robot.getIndexer().artifactColorArray[2]);

            // Update turret angle so that it always point to the goal
            robot.updateTurretAngle();
//
//            //read oblisk if not ready yet
//            if (!RobotStaticValuesClass.obliskReady){
//                robot.getMotif();
//            }

            debugManager.update();
        }
    }
}
