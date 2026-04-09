package org.firstinspires.ftc.teamcode.common;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.ftc.InvertedFTCCoordinates;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.common.subsystems.DriveBase;
import org.firstinspires.ftc.teamcode.common.subsystems.Hud;
import org.firstinspires.ftc.teamcode.common.subsystems.Indexer;
import org.firstinspires.ftc.teamcode.common.subsystems.Intake;
import org.firstinspires.ftc.teamcode.common.subsystems.Launcher;
import org.firstinspires.ftc.teamcode.common.util.ArtifactColor;
import org.firstinspires.ftc.teamcode.common.util.DebugManager;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import com.qualcomm.hardware.lynx.LynxModule;

import java.util.List;
import java.util.LinkedList;

@Configurable
public class Robot {

    private static final Logger log = LoggerFactory.getLogger(Robot.class);
    private DriveBase driveBase;
    private Indexer indexer;
    private Launcher launcher;
    private Intake intake;
    private Hud hud;
    private Limelight3A limelight;
    private List<LynxModule> allHubs;

    private ElapsedTime timeSinceIndex = new ElapsedTime();
    private ElapsedTime timeSinceKick = new ElapsedTime();

    private ElapsedTime timeSinceElevate = new ElapsedTime();

    private ElapsedTime timeSinceKickReset  = new ElapsedTime();
    private ElapsedTime reverseIntakeTimer  = new ElapsedTime();

    public ElapsedTime signalAllShotTimer = new ElapsedTime();

    //indicators for driver
    public boolean intake3Balls = false; //Picked up all three balls
    public boolean intake1Ball = false; //Picked up one ball
    public boolean signalPlayer2 = false;
    private boolean safeToStop = true; //if kicker is down
    public static boolean enableDriftCorrection = false; // use vision to correct pp drift in tele
    private boolean isRedSide = false;

    public boolean isRedSide() {
        return isRedSide;
    }


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

    public static final long WAIT_TIME_ELEVATOR = 300;

    // turret offset from the center of the robot
    public static final double TURRET_OFFSET = 1.679; //inches
    // blue goal x and y
    public static final double BLUE_X = -63.5; //inches
    public static final double BLUE_Y = -63.5; //inches
    // red goal x and y
    public static final double RED_X = -63.5; //inches
    public static final double RED_Y = 63.5; //inches


//    public final double LIMELIGHT_OFFSET = 17.4; //todo: update
//    public final double LIMELIGHT_HEIGHT_OFFSET = 436; //todo: update

    public static final double PINPOINT_B1_YAW_SCALAR = 1.0033595;

    public Robot(HardwareMap hardwareMap, Telemetry telemetry, boolean isRed) {
        // Create an instance of the hardware map and telemetry in the Robot class
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        debugManager = new DebugManager(telemetry, "launcher");
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

        // Enable bulk caching AFTER all subsystems init so their init loops get live reads
        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

//        limelight = hardwareMap.get(Limelight3A.class, "limelight");
//        limelight.pipelineSwitch(7);

//        limelight.start();

        // Initialize the map with calibration points.
        // Distances in cm, velocities as motor power (0.0 to 1.0)
        // Example values:

    }

    /** Call once at the top of every OpMode loop to refresh the bulk-read cache. */
    public void clearBulkCache() {
        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }
    }

    public enum AutoIntakeState {
        INIT,
        TURN_EMPTY_SLOT_TO_INTAKE,
        WAIT_FOR_BALL,
        POSITION_FOR_OUTTAKE,
        READY_TO_SHOOT
    }

    @Deprecated
    public enum LaunchBallKickerState {
        IDLE,
        INIT,
        TURN_TO_LAUNCH,
        KICK_BALL,
        RESET_KICKER,
        UPDATE_INDEXER,
        READY_TO_INTAKE
    }

    public enum LaunchBallState {
        IDLE,
        INIT,
        TURN_TO_LAUNCH,
        ELEVATE,
        UPDATE_INDEXER,
        READY_TO_INTAKE
    }

    public enum RobotInOutState {
        IDLE,
        INTAKE,
        OUTTAKE
    }
    /*
        LIMELIGHT PIPELINES:        TYPE:               STATUS:
            0: PURPLE               COLOR               USED
            1: YELLOW               COLOR               OPEN FOR CONLaunchBallStatesFIGURATION
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

    @Deprecated
    LaunchBallKickerState launchStateKicker = LaunchBallKickerState.INIT;

    LaunchBallState launchState = LaunchBallState.INIT;

    AutoIntakeState autoIntakeState = AutoIntakeState.INIT;
    RobotInOutState robotInOutState = RobotInOutState.IDLE;

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
                 debugManager.log("intakeWithIndexerTurn: INIT)");
                 if (indexer.checkEmptySlot()) {
                     debugManager.robot("Robot: found empty slot");
                     debugManager.log("RRobot: found empty slot");
                     if(indexer.turnEmptySlotToIntake()) {
                         intake.stopIntake();
                         autoIntakeState = AutoIntakeState.TURN_EMPTY_SLOT_TO_INTAKE;
                         break;
                     }
                     else {
                         autoIntakeState = AutoIntakeState.WAIT_FOR_BALL;
                         break;
                     }
                 } else {
                     //No empty slot
                     // - update color double check
                     // This line is removed to save time.
                     // indexer.updateColorAllSlots();
                     intake3Balls = true;
                     break;
                 }
             case TURN_EMPTY_SLOT_TO_INTAKE:
                 debugManager.log("Robot: TURN_EMPTY_SLOT_TO_INTAKE");
                 if (indexer.indexerFinishedTurning()) {
                     debugManager.robot("Robot: indexerFinishedTurning");
                     intake.startIntake();
                     autoIntakeState = AutoIntakeState.WAIT_FOR_BALL;
                 }
                 break;
             case WAIT_FOR_BALL:
                 debugManager.log("Robot: WAIT_FOR_BALLt");

                 if (indexer.isBallAtIntakeFast()) {
                         debugManager.log("Robot: isBallAtIntakeFast");
                         intake1Ball = true;
                         intake.stopIntake();
                         autoIntakeState = AutoIntakeState.INIT;
                         break;
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
            launchStateKicker = LaunchBallKickerState.INIT;
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
            launchStateKicker = LaunchBallKickerState.INIT;
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
        if (launcher.isLauncherActive() && robotInOutState == RobotInOutState.OUTTAKE) {
            switch (launchState) {
                case INIT:

                    debugManager.log("shootAllBalls: INIT");
                    noArtifacts = false;
                    if(findNextBall()) {
                        debugManager.log("shootAllBalls: findNextBall");
                        launchState = LaunchBallState.TURN_TO_LAUNCH;
                    }
                    else {
                        if (!indexer.atIntake()) {
                            debugManager.log("shootAllBalls: !indexer.atIntake())");
                            indexer.positionForIntake();
                        }
                        launchState = LaunchBallState.READY_TO_INTAKE;
                        debugManager.log("shootAllBalls: INIT %s", launchState);
                        noArtifacts=true;
                    }
                    break;

                case TURN_TO_LAUNCH:
                    debugManager.log("shootAllBalls: TURN_TO_LAUNCH");
                    if (indexer.moveToOuttake()) {
                        debugManager.log("shootAllBalls: moveToOuttake()");
                    }
                    launchState = LaunchBallState.ELEVATE;
                    break;

                case ELEVATE:
                    debugManager.log("shootAllBalls: ELEVATE");
                    if (indexer.indexerFinishedTurning()) {
                        safeToStop = false;
                        launcher.elevateBall();
                        timeSinceElevate.reset();
                        launchState = LaunchBallState.UPDATE_INDEXER;
                    }
                    break;

                case UPDATE_INDEXER:
                    debugManager.log("shootAllBalls: UPDATE_INDEXER");
                    // check timer
                    //if (timeSinceElevate.milliseconds() > WAIT_TIME_ELEVATOR) {
                    if (launcher.isElevatorOutOfWay()) {
                        safeToStop = true;
                        indexer.updateAfterShoot();
                        shootQueue.removeFirst();
                        launchState = LaunchBallState.INIT;
                        debugManager.log("shootAllBalls: UPDATE_INDEXER set init");
                    }
                    break;

                case READY_TO_INTAKE:
                    debugManager.log("shootAllBalls: READY_TO_INTAKE");
                    if (indexer.indexerFinishedTurning()) {
                        //updateColorAllSlots();
                        if (indexer.findABall()) {
                            // for some reason, there is a ball in the indexer
                            shootOrder();
                        }
                        signalAllShotTimer.reset();
                        launchState = LaunchBallState.INIT;
                    }
                    break;

                default:
                    debugManager.log("shootAllBalls Unexpected");
                    throw new IllegalStateException("shootAllBalls Unexpected value: " + launchState);
            }
        }
    }

    public void shootAllBallsAuto() {

        debugManager.log("shootAllBalls");
        debugManager.log("0 color: %s", indexer.artifactColorArray[0]);
        debugManager.log("1 color: %s", indexer.artifactColorArray[1]);
        debugManager.log("2 color: %s", indexer.artifactColorArray[2]);
    // check to see if flywheel motors are running
        switch (launchStateKicker) {
            case INIT:
                debugManager.log("shootAllBalls: INIT");
                noArtifacts = false;
                if(indexer.findABall()) {
                    launchStateKicker = LaunchBallKickerState.TURN_TO_LAUNCH;
                    debugManager.log("shootAllBalls: findABall");
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
                launchStateKicker = LaunchBallKickerState.KICK_BALL;
                break;
            case KICK_BALL:
                debugManager.log("shootAllBalls: KICK_BALL");
                if (indexer.indexerFinishedTurning()) {
                    safeToStop = false;
                    launcher.kickBall();
                    timeSinceKick.reset();
                    launchStateKicker = LaunchBallKickerState.RESET_KICKER;
                    break;
                } else {
                    break;
                }
            case RESET_KICKER:
                debugManager.log("shootAllBalls: RESET_KICKER");
                if (timeSinceKick.milliseconds() > WAIT_TIME_KICKER_UP) {
                    launcher.resetKicker();
                    timeSinceKickReset.reset();
                    launchStateKicker = LaunchBallKickerState.UPDATE_INDEXER;
                    break;
                } else {
                    break;
                }
            case UPDATE_INDEXER:
                debugManager.log("Update_indexer");
                if (timeSinceKickReset.milliseconds() > WAIT_TIME_KICKER_DOWN) {
                    safeToStop = true;
                    indexer.updateAfterShoot();
                    launchStateKicker = LaunchBallKickerState.INIT;
                }
                break;
            case READY_TO_INTAKE:
                debugManager.log("Ready to Intake");
                if (indexer.indexerFinishedTurning()) {
                    updateColorAllSlots();
                    launchStateKicker = LaunchBallKickerState.INIT;
                }
                break;
            default:
                debugManager.log("Exception illegal");
                throw new IllegalStateException("shootAllBalls Unexpected value: " + launchStateKicker);
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
    public void updateShootingDistanceAngle(){
        //read the current pose
        double robotX = driveBase.getPinPointPosX();
        double robotY = driveBase.getPinPointPosY();
        //read the current robot heading
        double pinPointHeading = driveBase.getPinPointHeading();
        pinPointHeading = normalizeAngle(pinPointHeading);
        double robotHeading = Math.toDegrees(pinPointHeading);

        // offset turret x y from the center of the robot rotation
        double tx = robotX - Math.cos(pinPointHeading)*TURRET_OFFSET; /*- Math.sin(pinPointHeading) * TURRET_OFFSET;*/
        double ty = robotY - Math.sin(pinPointHeading)*TURRET_OFFSET;  /*- Math.cos(pinPointHeading) * TURRET_OFFSET;*/

        //calculate the relative angle of the turret to the robot
        double goalX;
        double goalY;
        // coordinates of the blue goal
        if (isRedSide) {
            goalX = RED_X;
            goalY = RED_Y;
        }
        else {
            goalX = BLUE_X;
            goalY = BLUE_Y;
        }

        // calculate vector to goal
        double deltaX = goalX - tx;
        double deltaY = goalY - ty;
        double distanceToGoal = Math.sqrt(Math.pow(deltaX, 2) + Math.pow(deltaY, 2));
        launcher.setShootingDistance(distanceToGoal);

        //calculates the angle in radians between the positive x-axis and a point
        double absoluteAngleRadians = Math.atan2(deltaY, deltaX);
        double absoluteAngleDegree = Math.toDegrees(absoluteAngleRadians);

        double relativeAngle;
        relativeAngle = (absoluteAngleDegree - robotHeading);
        if (isRedSide) {
            relativeAngle = normalizeAngle(relativeAngle);
        }
        else {
            relativeAngle = normalizeAngle(relativeAngle - 5); //off set on blue side
        }

        last_TurretAngle_Target = relativeAngle; // TODO need this in telemetry in main

        debugManager.robot("deltaX:", "%.2f", deltaX);
        debugManager.robot("deltaY:", "%.2f", deltaY);
        debugManager.robot("robotX:", "%.2f", robotX);
        debugManager.robot("robotY:", "%.2f", robotY);
        debugManager.robot("tx:", "%.2f", tx);
        debugManager.robot("ty:", "%.2f", ty);
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

        double tx = robotX - Math.cos(pinPointHeading) * TURRET_OFFSET;
        double ty = robotY - Math.sin(pinPointHeading) * TURRET_OFFSET;

        //calculate the relative angle of the turret to the robot
        double goalX;
        double goalY;
        // coordinates of the blue goal
        if (isRedSide) {
            goalX = RED_X;
            goalY = RED_Y;
        }
        else {
            goalX = BLUE_X;
            goalY = BLUE_Y;
        }


        // calculate vector to blue goal
        double deltaX = goalX - tx;
        double deltaY = goalY - ty;
        double distanceToGoal = Math.sqrt(Math.pow(deltaX, 2) + Math.pow(deltaY, 2));

        //calculates the angle in radians between the positive x-axis and a point
        double absoluteAngleRadians = Math.atan2(deltaY, deltaX);
        double absoluteAngleDegree = Math.toDegrees(absoluteAngleRadians);

        double relativeAngle;
        relativeAngle = (absoluteAngleDegree - robotHeading);

        if (isRedSide) {
            relativeAngle -= 3.5;
        }
        else {
            relativeAngle -= 5;
        }

        relativeAngle = normalizeAngle(relativeAngle);

//        debugManager.robot("deltaX:", "%.2f", deltaX);
//        debugManager.robot("deltaY:", "%.2f", deltaY);
//        debugManager.robot("robotX:", "%.2f", robotX);
//        debugManager.robot("robotY:", "%.2f", robotY);
//        debugManager.robot("absoluteAngleRadians:", "%.2f", absoluteAngleRadians);
//        debugManager.robot("absoluteAngleDegree:", "%.2f", absoluteAngleDegree);
//        debugManager.robot("pinPointHeading:", "%.2f", pinPointHeading);
//        debugManager.robot("relativeAngle:", "%.2f", relativeAngle);
//        debugManager.robot("distance to goal", "%.2f", distanceToGoal);

        launcher.autoUpdateTurretPID(relativeAngle);
    }

    double buffer = 10;


    // TODO check logic
    public double normalizeAngle (double angle) {

        if (lastRelativeHeading == -5000) {
            lastRelativeHeading = angle;
        }


        // TODO will this 'ever' happen more than once? (while?)
        while (angle > 180.0) {
            // TODO what about when exactly == 180?
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

    public void setRobotState(RobotInOutState state) {
        robotInOutState = state;
    }

    public RobotInOutState getRobotInOutState() {
        return robotInOutState;
    }

    public void setLaunchState(LaunchBallState state) {
        launchState = state;
    }

    public LaunchBallKickerState getLaunchStateKicker() {
        return launchStateKicker;
    }

    public void setAutoIntakeState(AutoIntakeState state) {
        autoIntakeState = state;
    }

    public AutoIntakeState getAutoIntakeStat() {
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
        launchStateKicker = LaunchBallKickerState.INIT;
        targetPattern = TargetPattern.NONE;
        shootOrder();
    }

    public void shootOrderMotif() {
        shootQueue.clear();
        launchStateKicker = LaunchBallKickerState.INIT;
        if (RobotStaticValuesClass.savedObelisk == RobotStaticValuesClass.Obelisk.GPP)
            targetPattern = TargetPattern.GPP;
        else if (RobotStaticValuesClass.savedObelisk == RobotStaticValuesClass.Obelisk.PGP)
            targetPattern = TargetPattern.PGP;
        else if (RobotStaticValuesClass.savedObelisk == RobotStaticValuesClass.Obelisk.PPG)
            targetPattern = TargetPattern.PPG;

        shootOrder();
    }

    public void shootOrderMotifOneOff() {
        shootQueue.clear();
        launchStateKicker = LaunchBallKickerState.INIT;
        if (RobotStaticValuesClass.savedObelisk == RobotStaticValuesClass.Obelisk.GPP)
            targetPattern = TargetPattern.PPG;
        else if (RobotStaticValuesClass.savedObelisk == RobotStaticValuesClass.Obelisk.PGP)
            targetPattern = TargetPattern.GPP;
        else if (RobotStaticValuesClass.savedObelisk == RobotStaticValuesClass.Obelisk.PPG)
            targetPattern = TargetPattern.PGP;

        shootOrder();
    }

    public void shootOrderMotifTwoOff() {
        shootQueue.clear();
        launchStateKicker = LaunchBallKickerState.INIT;
        if (RobotStaticValuesClass.savedObelisk == RobotStaticValuesClass.Obelisk.GPP)
            targetPattern = TargetPattern.PGP;
        else if (RobotStaticValuesClass.savedObelisk == RobotStaticValuesClass.Obelisk.PGP)
            targetPattern = TargetPattern.PPG;
        else if (RobotStaticValuesClass.savedObelisk == RobotStaticValuesClass.Obelisk.PPG)
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
