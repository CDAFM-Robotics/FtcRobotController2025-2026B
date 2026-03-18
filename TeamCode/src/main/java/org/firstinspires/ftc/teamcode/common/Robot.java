package org.firstinspires.ftc.teamcode.common;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.ftc.InvertedFTCCoordinates;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.common.subsystems.DriveBase;
import org.firstinspires.ftc.teamcode.common.subsystems.Hud;
import org.firstinspires.ftc.teamcode.common.subsystems.Indexer;
import org.firstinspires.ftc.teamcode.common.subsystems.Intake;
import org.firstinspires.ftc.teamcode.common.subsystems.Launcher;
import org.firstinspires.ftc.teamcode.common.util.ArtifactColor;
import org.firstinspires.ftc.teamcode.common.util.DebugManager;

import java.util.LinkedList;

@Configurable
public class Robot {

    private DriveBase driveBase;
    private Indexer indexer;
    private Launcher launcher;
    private Intake intake;
    private Hud hud;
    private Limelight3A limelight;

    private ElapsedTime timeSinceIndex = new ElapsedTime();
    private ElapsedTime timeSinceKick = new ElapsedTime();
    private ElapsedTime timeSinceKickReset  = new ElapsedTime();
    private ElapsedTime reverseIntakeTimer  = new ElapsedTime();

    //indicators for driver
    public boolean intake3Balls = false; //Picked up all three balls
    public boolean intake1Ball = false; //Picked up one ball
    private boolean safeToStop = true; //if kicker is down
    private boolean isRedSide = false;

    // PID targets for use in Auto
    public double last_TurretAngle_Target;
    public double last_PIDShootingPower_Target;

    private ArtifactColor ballColor = ArtifactColor.NONE;

    private HardwareMap hardwareMap;
    private Telemetry telemetry;
    private final DebugManager debugManager;

    //shooting order for the balls
    LinkedList<Integer> shootQueue = new LinkedList<>();
    enum TargetPattern {
        NONE,
        GPP,
        PGP,
        PPG
    }
    private TargetPattern targetPattern = TargetPattern.NONE;

    public static int WAIT_TIME_KICKER_UP = 180; // 140; //170; // 250; // 75 didn't shoot once  // was 175 // was 275 (SINGLE RB WHEEL)
    public static int WAIT_TIME_KICKER_DOWN = 80; // 45; // 80; // 150; // 75 didn't shoot once  // was 175 // was 275 (SINGLE RB WHEEL)

//    public final double LIMELIGHT_OFFSET = 17.4; //todo: update
//    public final double LIMELIGHT_HEIGHT_OFFSET = 436; //todo: update

    public Robot(HardwareMap hardwareMap, Telemetry telemetry, boolean isRed) {
        // Create an instance of the hardware map and telemetry in the Robot class
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        debugManager = new DebugManager(telemetry, "launcher");
        timeSinceIndex.startTime();
        timeSinceKick.startTime();
        timeSinceKickReset.startTime();
        reverseIntakeTimer.startTime();
        isRedSide = isRed;
        initializeSubsystems(isRed);
    }

    public void initializeSubsystems(boolean isRed) {
        // Create an instance of every subsystem in the Robot class
        this.driveBase = new DriveBase(this.hardwareMap, this.telemetry, isRed);
        this.indexer = new Indexer(this.hardwareMap, this.telemetry);
        this.launcher = new Launcher(this.hardwareMap, this.telemetry);
        this.intake = new Intake(this.hardwareMap, this.telemetry);
        //this.hud = new Hud(this.hardwareMap, this.telemetry);

//        limelight = hardwareMap.get(Limelight3A.class, "limelight");
//        limelight.pipelineSwitch(7);

//        limelight.start();

        // Initialize the map with calibration points.
        // Distances in cm, velocities as motor power (0.0 to 1.0)
        // Example values:
    /*
        LIMELIGHT PIPELINES:        TYPE:               STATUS:
            0: PURPLE               COLOR               USED
            1: YELLOW               COLOR               OPEN FOR CONFIGURATION
            2: BLUE                 COLOR               OPEN FOR CONFIGURATION
            3: APRIL_TAG            AprilTag            OPEN FOR CONFIGURATION
            4: MOTIF                AprilTag            USED
            5: RED_GOAL             AprilTag            USED
            6: BLUE_GOAL            AprilTag            USED
            7: OBELISK              AprilTag            USED

     */
        // read the target pattern from autonomous
        if (RobotStaticValuesClass.autoCompleted) {
            RobotStaticValuesClass.obliskReady = true;
        }
        else {
            // try to read the oblisk with limelight
//            getMotif();
        }
    }

    public enum AutoIntakeStates {
        INIT,
        TURN_EMPTY_SLOT_TO_INTAKE,
        WAIT_FOR_BALL,
        POSITION_FOR_OUTTAKE,
        READY_TO_SHOOT
    }

    public enum LaunchBallStates {
        IDLE,
        INIT,
        TURN_TO_LAUNCH,
        KICK_BALL,
        RESET_KICKER,
        UPDATE_INDEXER,
        READY_TO_INTAKE
    }

    public enum RobotInOutStates {
        IDLE,
        INTAKE,
        OUTTAKE
    }
    /*
        LIMELIGHT PIPELINES:        TYPE:               STATUS:
            0: PURPLE               COLOR               USED
            1: YELLOW               COLOR               OPEN FOR CONFIGURATION
            2: BLUE                 COLOR               OPEN FOR CONFIGURATION
            3: APRIL_TAG            AprilTag            OPEN FOR CONFIGURATION
            4: MOTIF                AprilTag            USED
            5: RED_GOAL             AprilTag            USED
            6: BLUE_GOAL            AprilTag            USED
            7: OBELISK              AprilTag            USED

     */
    public enum LLPipelines {
        PURPLE,
        YELLOW,
        BLUE,
        APRIL_TAG,
        MOTIF,
        RED_GOAL,
        BLUE_GOAL,
        OBELISK
    }

    LaunchBallStates launchState = LaunchBallStates.INIT;
    AutoIntakeStates autoIntakeState = AutoIntakeStates.INIT;
    RobotInOutStates robotInOutState = RobotInOutStates.IDLE;

    public DriveBase getDriveBase() {
        return driveBase;
    }

    public Indexer getIndexer() {
        return indexer;
    }

    public Launcher getLauncher() {
        return launcher;
    }

    public Intake getIntake() {
        return intake;
    }

    public Hud getHud() {
        return hud;
    }

     // Auto-Indexing for intake
     public void intakeWithIndexerTurn(){
         debugManager.robot("intakeWithIndexerTurn", "%s",autoIntakeState);

         switch (autoIntakeState) {
             case INIT:
                 RobotLog.d("intakeWithIndexerTurn: INIT)");
                 if (indexer.checkEmptySlot()) {
                     debugManager.robot("Robot: found empty slot", "");
                     debugManager.log("RRobot: found empty slot");
                     autoIntakeState = AutoIntakeStates.TURN_EMPTY_SLOT_TO_INTAKE;
                     break;
                 } else {
                     //No empty slot
                     // - update color double check
                     // This line is removed to save time.
                     indexer.updateColorAllSlots();
                     intake3Balls = true;
                     break;
                 }
             case TURN_EMPTY_SLOT_TO_INTAKE:
                 RobotLog.d("intakeWithIndexerTurn: TURN_EMPTY_SLOT_TO_INTAKE)");
                 indexer.turnEmptySlotToIntake();
                 timeSinceIndex.reset();
                 autoIntakeState = AutoIntakeStates.WAIT_FOR_BALL;
                 break;
             case WAIT_FOR_BALL:
                 debugManager.robot("Robot: WAIT_FOR_BALL", "");
                 debugManager.log("Robot: WAIT_FOR_BALLt");
                 if (indexer.indexerFinishedTurning()) {
                     // debugManager.robot("Robot: indexerFinishedTurning");
                     // removed to make intake faster
                     //indexer.updateColorAllSlots();
                     debugManager.log("Robot: indexerFinishedTurning");
                     if (indexer.isBallAtIntakeFast()) {
                         // debugManager.robot("Robot: isBallAtIntake");
                         debugManager.log("Robot: isBallAtIntake");
                         intake1Ball = true;
                         //Reading color in isBallAtIntake. No need to read here anymore
                         indexer.updateColorAtIntakeOnly();
                         autoIntakeState = AutoIntakeStates.INIT;
                         break;
                     }
                 }
                 break;
             default:
                 throw new IllegalStateException("intakeWithIndexerTurn Unexpected value: " + autoIntakeState);
         }
     }

    public Telemetry getTelemetry() {
        return telemetry;
    }

    public void startLaunchAGreenBall(){
        if(launcher.isLauncherActive()) {
            //debugManager.robot("stratLaunchAGreenBall");
            ballColor = ArtifactColor.GREEN;
            launchState = LaunchBallStates.INIT;
        }
        if (launcher.getKickerPosition() == launcher.POSITION_KICKER_SERVO_KICK_BALL) {
            launcher.resetKicker();
            timeSinceKickReset.reset();
        }
    }

    public void startLaunchAPurpleBall(){
        if(launcher.isLauncherActive()) {
            //debugManager.robot("stratLaunchAPupleBall");
            ballColor = ArtifactColor.PURPLE;
            launchState = LaunchBallStates.INIT;
        }
        if (launcher.getKickerPosition() == launcher.POSITION_KICKER_SERVO_KICK_BALL) {
            launcher.resetKicker();
            timeSinceKickReset.reset();
        }
    }

    public void launchAColorBall(){

            //debugManager.robot("launchAColorBall", ballColor);
            //debugManager.robot("color:", indexer.artifactColorArray[0]);
            //debugManager.robot("color:", indexer.artifactColorArray[1]);
            //debugManager.robot("color:", indexer.artifactColorArray[2]);

    }

    boolean noArtifacts = false;

    public void shootAllBalls() {
        //debugManager.robot("shootAllBalls");

//        debugManager.robot("color:", indexer.artifactColorArray[0]);
//        debugManager.robot("color:", indexer.artifactColorArray[1]);
//        debugManager.robot("color:", indexer.artifactColorArray[2]);

        // check to see if flywheel motors are running
        if(launcher.isLauncherActive() && robotInOutState == RobotInOutStates.OUTTAKE) {
            debugManager.log("shootAllBalls");
            debugManager.log("0 color: %s", indexer.artifactColorArray[0]);
            debugManager.log("1 color: %s", indexer.artifactColorArray[1]);
            debugManager.log("2 color: %s", indexer.artifactColorArray[2]);

                switch (launchState) {
                    case INIT:
                        noArtifacts = false;
                        //debugManager.robot("shootAllBalls: INIT");
                        debugManager.log("shootAllBalls: INIT");
                        if(findNextBall()) {
                            debugManager.log("shootAllBalls: findNextBall");
                            launchState = LaunchBallStates.TURN_TO_LAUNCH;
                            break;
                        }
                        else {
                            if (!indexer.atIntake()) {
                                debugManager.log("shootAllBalls: !indexer.atIntake())");
                                indexer.positionForIntake();
                            }
                                launchState = LaunchBallStates.READY_TO_INTAKE;
                            debugManager.log("shootAllBalls: INIT %s", launchState);
                                break;
                        }
                    case TURN_TO_LAUNCH:
                        //debugManager.robot("shootAllBalls: TURN_TO_LAUNCH");
                        debugManager.log("shootAllBalls: TURN_TO_LAUNCH");
                        if (indexer.moveToOuttake()) {
                            debugManager.log("shootAllBalls: moveToOuttake()");
                        }
                        launchState = LaunchBallStates.KICK_BALL;
                        break;
                    case KICK_BALL:
                        //debugManager.robot("shootAllBalls: KICK_BALL");
                        debugManager.log("shootAllBalls: KICK_BALL");
                        if (indexer.indexerFinishedTurning()) {
                            safeToStop = false;
                            launcher.kickBall();
                            timeSinceKick.reset();
                            launchState = LaunchBallStates.RESET_KICKER;
                            break;
                        } else {
                            break;
                        }
                    case RESET_KICKER:
                        //debugManager.robot("shootAllBalls: RESET_KICKER");
                        debugManager.log("shootAllBalls: RESET_KICKER");
                        if (timeSinceKick.milliseconds() > WAIT_TIME_KICKER_UP) {
                            launcher.resetKicker();
                            timeSinceKickReset.reset();
                            launchState = LaunchBallStates.UPDATE_INDEXER;
                            break;
                        } else {
                            break;
                        }
                    case UPDATE_INDEXER:
                        //debugManager.robot("shootAllBalls: UPDATE_INDEXER");
                        debugManager.log("shootAllBalls: UPDATE_INDEXER");
                        if (timeSinceKickReset.milliseconds() > WAIT_TIME_KICKER_DOWN) {
                            safeToStop = true;
                            indexer.updateAfterShoot();
                            shootQueue.removeFirst();
                            launchState = LaunchBallStates.INIT;
                            debugManager.log("shootAllBalls: UPDATE_INDEXER set init");
                        }
                        break;
                    case READY_TO_INTAKE:
                        debugManager.log("shootAllBalls: READY_TO_INTAKE");
                        if (indexer.indexerFinishedTurning()) {
                            updateColorAllSlots();
                            if (indexer.findABall()) {
                                // for some reason, there is a ball in the indexer
                                shootOrder();
                            }
                            launchState = LaunchBallStates.INIT;
                        }
                        break;
                    default:
                        debugManager.log("shootAllBalls Unexpected");
                        throw new IllegalStateException("shootAllBalls Unexpected value: " + launchState);
                }
        }
    }

    public void shootAllBallsOrder() {}


    public void shootAllBallsAuto() {

        debugManager.log("shootAllBalls");
        debugManager.log("0 color: %s", indexer.artifactColorArray[0]);
        debugManager.log("1 color: %s", indexer.artifactColorArray[1]);
        debugManager.log("2 color: %s", indexer.artifactColorArray[2]);
    // check to see if flywheel motors are running
        switch (launchState) {
            case INIT:
                debugManager.log("shootAllBalls: INIT");
                noArtifacts = false;
                if(indexer.findABall()) {
                    debugManager.log("shootAllBalls: findABall");
                    launchState = LaunchBallStates.TURN_TO_LAUNCH;
                    break;
                }
                else {
                    debugManager.log("shootAllBalls: NOT findABall");
                    noArtifacts = true;
                    break;
                }
            case TURN_TO_LAUNCH:
                debugManager.log("shootAllBalls: TURN_TO_LAUNCH");
                indexer.moveToOuttake();
                launchState = LaunchBallStates.KICK_BALL;
                break;
            case KICK_BALL:
                debugManager.log("shootAllBalls: KICK_BALL");
                if (indexer.indexerFinishedTurning()) {
                    safeToStop = false;
                    launcher.kickBall();
                    timeSinceKick.reset();
                    launchState = LaunchBallStates.RESET_KICKER;
                    break;
                } else {
                    break;
                }
            case RESET_KICKER:
                debugManager.log("shootAllBalls: RESET_KICKER");
                if (timeSinceKick.milliseconds() > WAIT_TIME_KICKER_UP) {
                    launcher.resetKicker();
                    timeSinceKickReset.reset();
                    launchState = LaunchBallStates.UPDATE_INDEXER;
                    break;
                } else {
                    break;
                }
            case UPDATE_INDEXER:
                debugManager.log("Update_indexer");
                if (timeSinceKickReset.milliseconds() > WAIT_TIME_KICKER_DOWN) {
                    safeToStop = true;
                    indexer.updateAfterShoot();
                    launchState = LaunchBallStates.INIT;
                }
                break;
            case READY_TO_INTAKE:
                debugManager.log("Ready to Intake");
                if (indexer.indexerFinishedTurning()) {
                    updateColorAllSlots();
                    launchState = LaunchBallStates.INIT;
                }
                break;
            default:
                debugManager.log("Exception illegal");
                throw new IllegalStateException("shootAllBalls Unexpected value: " + launchState);
        }
    }

    public boolean isNoArtifacts() {
        return noArtifacts;
    }

    public void resetIndexer() {
    }

    public void robotStopIntake(){
        // This line cause intake color mistakes. To be investigated
        // indexer.updateBallColors();
        intake.stopIntake();
    }

    public Boolean isIntake3Balls () {
        return intake3Balls;
    }

    public void setIntake3BallsOff() {
        intake3Balls = false;
    }

    public Boolean isIntake1Ball () {
        return intake1Ball;
    }

    public void setIntake1BallOff() {
        intake1Ball = false;
    }

    public void updateColorAllSlots() {
        indexer.updateColorAllSlots();
    }

    private double lastRelativeHeading = -5000; // Just to check whether it has been set yet

    //updating the turret every loop
    public void updateTurretAngle(){
        //read the current pose
        double robotX = driveBase.getPinPointPosX();
        double robotY = driveBase.getPinPointPosY();
        //read the current robot heading
        double pinPointHeading = driveBase.getPinPointHeading();
        pinPointHeading = normalizeAngle(pinPointHeading);
        double robotHeading = Math.toDegrees(pinPointHeading);

        //calculate the relative angle of the turret to the robot
        double blueGoalX;
        double blueGoalY;
        // coordinates of the blue goal
        if (isRedSide) {
            blueGoalX = -64;
            blueGoalY = 64;
        }
        else {
            blueGoalX = -64;
            blueGoalY = -64;
        }


        // calculate vector to blue goal
        double deltaX = blueGoalX - robotX;
        double deltaY = blueGoalY - robotY;
        double distanceToGoal = Math.sqrt(Math.pow(deltaX, 2) + Math.pow(deltaY, 2));
        launcher.setShootingDistance(distanceToGoal);

        //calculates the angle in radians between the positive x-axis and a point
        double absoluteAngleRadians = Math.atan2(deltaY, deltaX);
        double absoluteAngleDegree = Math.toDegrees(absoluteAngleRadians);

        double relativeAngle;
        if (isRedSide) {
            relativeAngle = (absoluteAngleDegree - robotHeading);
        }
        else {
            relativeAngle = (absoluteAngleDegree - robotHeading) + 2; //off set on blue side
        }

        relativeAngle = normalizeAngle(relativeAngle);

        debugManager.robot("deltaX:", "%.2f", deltaX);
        debugManager.robot("deltaY:", "%.2f", deltaY);
        debugManager.robot("robotX:", "%.2f", robotX);
        debugManager.robot("robotY:", "%.2f", robotY);
        debugManager.robot("absoluteAngleRadians:", "%.2f", absoluteAngleRadians);
        debugManager.robot("absoluteAngleDegree:", "%.2f", absoluteAngleDegree);
        debugManager.robot("pinPointHeading:", "%.2f", pinPointHeading);
        debugManager.robot("relativeAngle:", "%.2f", relativeAngle);
        debugManager.robot("distance to goal","%.2f", distanceToGoal);

        launcher.setTurretRelativeAngle(relativeAngle);
    }

    //updating the turret every loop
    public void updateTurretAngleAuto(){
        //read the current pose
        Pose FTCPose = new Pose();
        FTCPose = InvertedFTCCoordinates.INSTANCE.convertFromPedro(new Pose(driveBase.getPinPointPosX(), driveBase.getPinPointPosY(), driveBase.getPinPointHeading()));
        double robotX = FTCPose.getX();
        double robotY = FTCPose.getY();
        //read the current robot heading
        double pinPointHeading = FTCPose.getHeading();
        pinPointHeading = normalizeAngle(pinPointHeading);
        double robotHeading = Math.toDegrees(pinPointHeading);

        //calculate the relative angle of the turret to the robot
        double blueGoalX;
        double blueGoalY;
        // coordinates of the blue goal
        if (isRedSide) {
            blueGoalX = -64;
            blueGoalY = 64;
        }
        else {
            blueGoalX = -64;
            blueGoalY = -64;
        }


        // calculate vector to blue goal
        double deltaX = blueGoalX - robotX;
        double deltaY = blueGoalY - robotY;
        double distanceToGoal = Math.sqrt(Math.pow(deltaX, 2) + Math.pow(deltaY, 2));
        launcher.setShootingDistance(distanceToGoal);

        //calculates the angle in radians between the positive x-axis and a point
        double absoluteAngleRadians = Math.atan2(deltaY, deltaX);
        double absoluteAngleDegree = Math.toDegrees(absoluteAngleRadians);

        double relativeAngle;
        if (isRedSide) {
            relativeAngle = (absoluteAngleDegree - robotHeading);
        }
        else {
            relativeAngle = (absoluteAngleDegree - robotHeading) + 6; //off set on blue side
        }

        relativeAngle = normalizeAngle(relativeAngle);

        debugManager.robot("deltaX:", "%.2f", deltaX);
        debugManager.robot("deltaY:", "%.2f", deltaY);
        debugManager.robot("robotX:", "%.2f", robotX);
        debugManager.robot("robotY:", "%.2f", robotY);
        debugManager.robot("absoluteAngleRadians:", "%.2f", absoluteAngleRadians);
        debugManager.robot("absoluteAngleDegree:", "%.2f", absoluteAngleDegree);
        debugManager.robot("pinPointHeading:", "%.2f", pinPointHeading);
        debugManager.robot("relativeAngle:", "%.2f", relativeAngle);
        debugManager.robot("distance to goal", "%.2f", distanceToGoal);

        launcher.setTurretRelativeAngle(relativeAngle);
    }

    double buffer = 10;

    public double normalizeAngle (double angle) {

        if (lastRelativeHeading == -5000) {
            lastRelativeHeading = angle;
        }


        while (angle > 180.0) {
            if (angle > 180 && angle < 180 + buffer && lastRelativeHeading > 0) {
                lastRelativeHeading = angle;
                return angle;
            }
            angle -= 360.0;
        }
        while (angle < -180.0) {
            if (angle < -180 && angle > -180 - buffer && lastRelativeHeading < 0) {
                lastRelativeHeading = angle;
                return angle;
            }
            angle += 360.0;
        }

        lastRelativeHeading = angle;
        return angle;
    }

    public boolean isSafeToStopOuttake() {
        return safeToStop;
    }

    public void setRobotState(RobotInOutStates state) {
        robotInOutState = state;
    }

    public RobotInOutStates getRobotInOutState() {
        return robotInOutState;
    }

    public void setLaunchState(LaunchBallStates state) {
        launchState = state;
    }

    public LaunchBallStates getLaunchState() {
        return launchState;
    }

    public void setAutoIntakeState(AutoIntakeStates state) {
        autoIntakeState = state;
    }

    public AutoIntakeStates getAutoIntakeStat() {
        return autoIntakeState;
    }

    public void shootOrder() {
        indexer.buildShootQueueNoColor(shootQueue);

        if (shootQueue.size() >= 2) {
            if (targetPattern == TargetPattern.GPP) {
                int index = shootQueue.get(0);
                if (indexer.artifactColorArray[index] != ArtifactColor.GREEN) {
                    for (int i=0; i < shootQueue.size(); i++) {
                        if (indexer.artifactColorArray[shootQueue.get(i)] == ArtifactColor.GREEN) {
                            int item = shootQueue.remove(i);  // remove from old position
                            shootQueue.add(0, item);
                            break;
                        }
                        else if (indexer.artifactColorArray[shootQueue.get(i)] == ArtifactColor.UNKNOWN) {
                            int item = shootQueue.remove(i);  // remove from old position
                            shootQueue.add(0, item);
                            break;
                        }
                    }
                }
            }
            else if (targetPattern == TargetPattern.PGP) {
                int index = shootQueue.get(1);
                if (indexer.artifactColorArray[index] != ArtifactColor.GREEN) {
                    for (int i=0; i < shootQueue.size(); i++) {
                        if (indexer.artifactColorArray[shootQueue.get(i)] == ArtifactColor.GREEN) {
                            int item = shootQueue.remove(i);  // remove from old position
                            shootQueue.add(1, item);
                            break;
                        }
                        else if (indexer.artifactColorArray[shootQueue.get(i)] == ArtifactColor.UNKNOWN) {
                            int item = shootQueue.remove(i);  // remove from old position
                            shootQueue.add(1, item);
                            break;
                        }
                    }
                }

            }
            else if (targetPattern == TargetPattern.PPG) {
                if (shootQueue.size() >= 3) {
                    int index = shootQueue.get(2);
                    if (indexer.artifactColorArray[index] != ArtifactColor.GREEN) {
                        for (int i = 0; i < shootQueue.size(); i++) {
                            if (indexer.artifactColorArray[shootQueue.get(i)] == ArtifactColor.GREEN) {
                                int item = shootQueue.remove(i);  // remove from old position
                                shootQueue.add(2, item);
                                break;
                            } else if (indexer.artifactColorArray[shootQueue.get(i)] == ArtifactColor.UNKNOWN) {
                                int item = shootQueue.remove(i);  // remove from old position
                                shootQueue.add(2, item);
                                break;
                            }
                        }
                    }
                }

            }
        }
        for (int value :shootQueue) {
            debugManager.robot("shootOrder:", "%d", value);
        }
    }

    public void shootOrderNone() {
        shootQueue.clear();
        launchState = LaunchBallStates.INIT;
        targetPattern = TargetPattern.NONE;
        shootOrder();
    }

    public void shootOrderMotif() {
        shootQueue.clear();
        launchState = LaunchBallStates.INIT;
        if (RobotStaticValuesClass.savedOblisk == RobotStaticValuesClass.Oblisk.GPP)
            targetPattern = TargetPattern.GPP;
        else if (RobotStaticValuesClass.savedOblisk == RobotStaticValuesClass.Oblisk.PGP)
            targetPattern = TargetPattern.PGP;
        else if (RobotStaticValuesClass.savedOblisk == RobotStaticValuesClass.Oblisk.PPG)
            targetPattern = TargetPattern.PPG;

        shootOrder();
    }

    public void shootOrderMotifOneOff() {
        shootQueue.clear();
        launchState = LaunchBallStates.INIT;
        if (RobotStaticValuesClass.savedOblisk == RobotStaticValuesClass.Oblisk.GPP)
            targetPattern = TargetPattern.PPG;
        else if (RobotStaticValuesClass.savedOblisk == RobotStaticValuesClass.Oblisk.PGP)
            targetPattern = TargetPattern.GPP;
        else if (RobotStaticValuesClass.savedOblisk == RobotStaticValuesClass.Oblisk.PPG)
            targetPattern = TargetPattern.PGP;

        shootOrder();
    }

    public void shootOrderMotifTwoOff() {
        shootQueue.clear();
        launchState = LaunchBallStates.INIT;
        if (RobotStaticValuesClass.savedOblisk == RobotStaticValuesClass.Oblisk.GPP)
            targetPattern = TargetPattern.PGP;
        else if (RobotStaticValuesClass.savedOblisk == RobotStaticValuesClass.Oblisk.PGP)
            targetPattern = TargetPattern.PPG;
        else if (RobotStaticValuesClass.savedOblisk == RobotStaticValuesClass.Oblisk.PPG)
            targetPattern = TargetPattern.GPP;

        shootOrder();
    }

    public boolean findNextBall() {
        debugManager.robot("findNextBall", "");
        for (int value :shootQueue) {
            debugManager.robot("shootOrder: ", "%d",value);
        }

        if ( !shootQueue.isEmpty() ) {
            debugManager.robot("findNextBall:!shootQueue.isEmpty()", "");
            indexer.setNextShootSlot(shootQueue.getFirst());
            //shootQueue.removeFirst();
            return true;
        }
        return false;
    }
//
//    public void getMotif() {
//        LLResult result = limelight.getLatestResult();
//        if (result.isValid()) {
//            List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
//            if (fiducialResults != null) {
//                for (LLResultTypes.FiducialResult fr : fiducialResults) {
//                    if (fr.getFiducialId() == 21) {
//                        RobotStaticValuesClass.savedOblisk = RobotStaticValuesClass.Oblisk.GPP;
//                        RobotStaticValuesClass.obliskReady = true;
//                    }
//                    else if (fr.getFiducialId() == 22) {
//                        RobotStaticValuesClass.savedOblisk = RobotStaticValuesClass.Oblisk.PGP;
//                        RobotStaticValuesClass.obliskReady = true;
//                    }
//                    else if (fr.getFiducialId() == 23) {
//                        RobotStaticValuesClass.savedOblisk = RobotStaticValuesClass.Oblisk.PPG;
//                        RobotStaticValuesClass.obliskReady = true;
//                    }
//                }
//            }
//        }
//        RobotLog.d("RobotStaticValuesClass.obliskReady %s", RobotStaticValuesClass.obliskReady);
//    }
//
//    public void setTagPipeline(int pipeline) {
//        limelight.pipelineSwitch(pipeline);
//    }

}
