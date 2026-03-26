package org.firstinspires.ftc.teamcode.teleop;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.field.Style;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.ftc.InvertedFTCCoordinates;
import com.pedropathing.ftc.PoseConverter;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.common.Robot;
import org.firstinspires.ftc.teamcode.common.subsystems.Hud;
import org.firstinspires.ftc.teamcode.common.util.DebugManager;
import org.firstinspires.ftc.teamcode.common.util.Drawing;

@Configurable
@TeleOp(name = "BLUE Bot2 ", group = "0teleop")
public class DriverControlWithIndexerBlueTeleOp extends LinearOpMode {
    public boolean isRedSide = false;

    // Make a local HUD
    private Hud hud;

    // ---- Loop throttle ----
    private int loopCount = 0;
    private static final int READ_EVERY_N_LOOPS = 10;
    private boolean colorConfirmed = false;

    private LLResult result;

    // TODO add Data to Panels
    static TelemetryManager telemetryM;

    // TODO adding pose for LL<->Pinpoint Sync
    Pose3D botpose_mt2 ;


    @Override
    public void runOpMode() throws InterruptedException {

        // TODO Add Data to Dashboard Start
        // FtcDashboard dashboard = FtcDashboard.getInstance();
        // telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        // TODO Panels telemetry and robot drawing
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        Drawing.init();

        // One line to set up — pass your telemetry and a tag name
        DebugManager debugManager = new DebugManager(telemetry, "TELEOP");
        // ── Toggle these for competition vs. development ────────────
        // ─── Master switches ────────────────────────────────────────

        debugManager.TELEMETRY_ENABLED = false;
        debugManager.ROBOT_LOG_ENABLED = true;

        // Individual Telemetry flags
        debugManager.LOG_DRIVEBASE  = false;
        debugManager.LOG_PINPOINT   = false;
        debugManager.LOG_VISION     = false;
        debugManager.LOG_LAUNCHER   = false;
        debugManager.LOG_SPINDEXER  = true;
        debugManager.LOG_INTAKE     = false;
        debugManager.LOG_HUD        = false;
        debugManager.LOG_ROBOT      = true;


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


        // TODO Adding localization Pipeline
        robot.getLauncher().setLimelightPipeline((Robot.LLPipelines.APRIL_TAG.ordinal()));


        hud = new Hud(hardwareMap, telemetry);

        debugManager.update();

        waitForStart();

        while (opModeIsActive()){

            // TODO LOCALIZATION UPDATE
            robot.getLauncher().getLimeiight().updateRobotOrientation(Math.toDegrees(robot.getDriveBase().getPinPointHeading()));

            result = robot.getLauncher().getLimeiight().getLatestResult();
            if (result != null && result.isValid()) {
                botpose_mt2 = result.getBotpose_MT2();
                if (botpose_mt2 != null) {
                    double x = botpose_mt2.getPosition().toUnit(DistanceUnit.INCH).x;
                    double y = botpose_mt2.getPosition().toUnit(DistanceUnit.INCH).y;
                    telemetryM.addData("MT2 Location:", "(" + x + ", " + y + ")");
                }
            }


            // TODO Check for Drift and Update the PINPOINT
            // code here
            // TODO end of LOCALIZATION FIX

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


            if (!robot.getDriveBase().kickStandIsSet) {
                robot.getDriveBase().setMotorPowers(gamepad1.left_stick_x, -gamepad1.left_stick_y, -gamepad1.right_stick_x, driveSpeed, fieldCentric);
            }

            // Kickstand control
            if (currentGamepad1.a != previousGamepad1.a) {
                robot.getDriveBase().setMotorPowers(0, 0, 0, driveSpeed, fieldCentric);
                robot.getDriveBase().setKickStand();
                sleep(500);
                if (robot.getDriveBase().kickStandIsSet) {

                    robot.getDriveBase().spinFrontWheels();
                }
                //robot.getDriveBase().setKickStandLight();
            }

            if (currentGamepad1.b != previousGamepad1.b) {
                robot.getDriveBase().resetKickStand();
                //robot.getDriveBase().resetKickStandLight();
            }

            // Intake Balls. Add isSafeToStop()
            if (currentGamepad1.right_trigger != 0.0
                && robot.isSafeToStopOuttake()) {
                // telemetry.addLine("gamepad 1 right trigger");
                // Robot entering intake state
                if (robot.getRobotInOutState() != Robot.RobotInOutState.INTAKE) {
                    robot.setRobotState(Robot.RobotInOutState.INTAKE);
                    //reset intake state
                    robot.setAutoIntakeState(Robot.AutoIntakeState.INIT);
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
                robot.setRobotState(Robot.RobotInOutState.IDLE);
                colorConfirmed = false;
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

            robot.getLauncher().updateElevator();

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
                && robot.getRobotInOutState() == Robot.RobotInOutState.IDLE) {
                    robot.setRobotState(Robot.RobotInOutState.OUTTAKE);
                    robot.setLaunchState(Robot.LaunchBallState.INIT);
            }

            if ((currentGamepad2.right_trigger != 0
                || currentGamepad2.left_trigger != 0
                || currentGamepad2.right_bumper
                || currentGamepad2.left_bumper)
                && robot.getRobotInOutState() == Robot.RobotInOutState.OUTTAKE) {
                robot.shootAllBalls();
            }

            if (currentGamepad2.right_trigger == 0
                && currentGamepad2.left_trigger == 0
                && !currentGamepad2.right_bumper
                && !currentGamepad2.left_bumper
                && !robot.isSafeToStopOuttake()) {
                robot.shootAllBalls();
            }

            if ((currentGamepad2.right_trigger == 0
                && currentGamepad2.left_trigger == 0
                && !currentGamepad2.right_bumper
                && !currentGamepad2.left_bumper)
                && robot.isSafeToStopOuttake()) {
                if (robot.getRobotInOutState() == Robot.RobotInOutState.OUTTAKE) {
                    robot.setRobotState(Robot.RobotInOutState.IDLE);
                    colorConfirmed = false;
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
            telemetryM.addData("Velocity", robot.getLauncher().getLauncherVelocity());
            // telemetryM.update(telemetry);

            debugManager.addData("TeleOp RobotInOutState:", "%s", robot.getRobotInOutState());
            // Update ball colors every 20 loops if the robot is in idle
            if (robot.getRobotInOutState() == Robot.RobotInOutState.IDLE
                && !colorConfirmed) {
                if (robot.getIndexer().axonAtIntake()) {
                    loopCount++;
                    // ---- Read sensors every N loops ----
                    if (loopCount % READ_EVERY_N_LOOPS == 0) {
                        if (robot.getIndexer().confirmColorMatch() ) {
                            colorConfirmed = true;
                            if (robot.getIndexer().countArtifacts() == 3) {
                                //turn to output
                                debugManager.addData("count %d", robot.getIndexer().countArtifacts());
                                robot.getIndexer().positionForOuttake();
                            }
                        }
                        else {
                            colorConfirmed = false;
                        }
                    }
                }
            }


            // TODO REMOVE FOR COMP !
            robot.getLauncher().setLiftMotorPIDFCoefficients();


            // turn to output after three balls are confirmed for three times

            // Refresh the indicator lights
            hud.setBalls(robot.getIndexer().artifactColorArray[0], robot.getIndexer().artifactColorArray[1],robot.getIndexer().artifactColorArray[2]);
            hud.UpdateBallUI();

            // TODO Add timing Log at end of loop
            debugManager.log("Blue TeleOp c0: %s c1: %s c2: %s",
                    robot.getIndexer().artifactColorArray[0],
                    robot.getIndexer().artifactColorArray[1],
                    robot.getIndexer().artifactColorArray[2]);

            // Update turret angle so that it always point to the goal
            robot.updateShootingDistanceAngle();
//
//            //read oblisk if not ready yet
//            if (!RobotStaticValuesClass.obliskReady){
//                robot.getMotif();
//            }

            debugManager.update();

            // TODO add panels telem
            telemetryM.update();

            // TODO update drawing in panels
            // drawOnlyCurrent();
            try{
                Drawing.drawRobot(PoseConverter.pose2DToPose(robot.getDriveBase().getPinPointPose(), InvertedFTCCoordinates.INSTANCE));
                if (botpose_mt2 != null) {
                    // Drawing.drawRobot(PoseConverter.pose2DToPose(new Pose2D(DistanceUnit.INCH, botpose_mt2.getPosition().x, botpose_mt2.getPosition().y, AngleUnit.DEGREES, botpose_mt2.getOrientation().getYaw()), InvertedFTCCoordinates.INSTANCE), new Style("", "Red", 0.1));
                    Drawing.drawRobot(PoseConverter.pose2DToPose(new Pose2D(DistanceUnit.METER, botpose_mt2.getPosition().x, botpose_mt2.getPosition().y, AngleUnit.DEGREES, botpose_mt2.getOrientation().getYaw()), InvertedFTCCoordinates.INSTANCE), new Style("", "Red", 0.5));
                }
                Drawing.sendPacket();
            } catch (Exception e) {
                throw new RuntimeException("Drawing failed" +e);
            }

        }
    }
}
