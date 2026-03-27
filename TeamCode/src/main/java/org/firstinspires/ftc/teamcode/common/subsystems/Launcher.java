package org.firstinspires.ftc.teamcode.common.subsystems;

import static java.lang.Thread.currentThread;
import static java.lang.Thread.sleep;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;

import com.pedropathing.ftc.InvertedFTCCoordinates;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.autonomous.tasks.AutoTaskMaker;
import org.firstinspires.ftc.teamcode.common.Robot;
import org.firstinspires.ftc.teamcode.common.RobotStaticValuesClass;
import org.firstinspires.ftc.teamcode.common.util.ArtifactColor;
import org.firstinspires.ftc.teamcode.common.util.DebugManager;
import org.firstinspires.ftc.teamcode.common.util.InterpolationTable;
import org.firstinspires.ftc.teamcode.common.util.RunTimeoutAction;
import org.firstinspires.ftc.teamcode.common.util.WaitUntilAction;

import java.util.Arrays;
import java.util.List;
import java.util.Map;
import java.util.TreeMap;

@Config
@Configurable
public class Launcher {


    HardwareMap hardwareMap;
    Telemetry telemetry;
    private final DebugManager debugManager;

    DcMotorEx launcherMotor1;
    DcMotorEx launcherMotor2;
    public double launchPower;
    double launcherVelocity;
    double hoodPosition;
    double kickerPosition;
    DcMotorEx elevatorMotor;
    TouchSensor elevatorLimitSwitch;

    public Servo kickerServo;
    CRServo turretServo;
    Servo hoodServo;
    AnalogInput RSFeedback;

    // turret servo
    CRServo launcherServo;
    AnalogInput launcherAnalogInput;
    TouchSensor turretLimitSwitch;

    public Limelight3A limelight;

    public void setLimelightPipeline(int ordinal) {
        limelight.pipelineSwitch(ordinal);
    }
    public Limelight3A getLimeiight() { return this.limelight; }

    public double getCurrentAngleOffset() {
        return currentAngleOffset;
    }

    public void setCurrentAngleOffset(double angleOffset) {
        currentAngleOffset = angleOffset;
    }
    public enum QuadrantRotatorServo{
        POSITIVE, NEGATIVE, ZERO
    }

    QuadrantRotatorServo currentQuadrant;

    double lastServoPosition;

    public static final double POSITION_KICKER_SERVO_KICK_BALL = 0.06; // 0.16; // 0.26 reset due to kicker swap
    public static final double POSITION_KICKER_SERVO_INIT = 0.50; // 0.51; reset due to kicker swap (probably clock error on servo horn)
    public static final double POSITION_TUREET_SERVO_INIT = 0.5;

    // Hood Servo
    public static final double POSITION_HOOD_SERVO_INIT = 0.1;
    public static final double POSITION_HOOD_SERVO_HIGH = 0.1;
    public static final double POSITION_HOOD_SERVO_LOW = 1.0;

    public static final double LAUNCH_POWER_FAR = 0.9;
    public static final double LAUNCH_POWER_NEAR= 0.8;
    public static final double LAUNCH_POWER_FULL= 1.0;
    public static final double LAUNCH_POWER_LOW=0.3;   // TODO find lowest valuable power and set this
    public static final double LAUNCH_VELOCITY_FAR = 2200; // was: 6000 (wrong, 2200tps/28ppr*60 rpm 4714 rpm )
    public static final double LAUNCH_VELOCITY_NEAR= 1300;
    public static final double LAUNCH_VELOCITY_FULL= 2200;
    public static final double LAUNCH_VELOCITY_LOW= 1060;   // TODO find lowest valuable power and set this


    //rotate autoaim PID Constants
    private double rotateIntegralSum = 0.0;
    public static double rotateKp = 0.1;
    public static double rotateKi = 0;
    public static double rotateKd = 0;
    public static double rotateKf = 0;

    // Teleop AutoAIM PID Constants
    public static double aimKp = 0.016; // 0.02
    public static double aimKi = 0.006; // 0.01
    public static double aimKd = 1.0; // 1.1 // 0.0055
    public static int aimTimeout = 650; // 800
    public static double powerStatic = 0.064; // 0.05
    public static double aimErrorTolerance = 0; // 3
    private double lastTime;
    private double integralSum = 0.0;
    // Limits for integral sum to prevent windup
    private double integralSumMax = 1.0;
    private double integralSumMin = -1.0;
    private double lastError = 0.0;

    public static double shootKp = 250; //25;
    public static double shootKi = 1.5;
    public static double shootKd = 10; //4;
    public static double shootKf = 1.1;
    public static double targetVelocity;
    public static double currentVelocity;
    private double shootingDistance;

    // Use a table for interpolation
    private InterpolationTable shootingTable;

    // Bot2 Turret control variables
    public static double turretkF = 0.11; // 19m26 was 0.10
    public static double turretkP = 0.004; // 19m26 was 0.005
    public static double turretkI = 0;
    public static double turretkD = 0.00002; // 19m26 was 0.000005

    // Positional PID
    public static double elevatorKp = 0;
    public static double elevatorKi = 0;
    public static double elevatorKd = 0;
    // public static double elevatorKf = 0;

    public static double lastelevatorKp = 0;
    public static double lastelevatorKi = 0;
    public static double lastelevatorKd = 0;
    // public static double lastelevatorKf = 0;


    // Velocity PID
    public static double elevatorVelKp = 0;
    public static double elevatorVelKi = 0;
    public static double elevatorVelKd = 0;
    public static double elevatorVelKf = 0;

    public static double lastelevatorVelKp = 0;
    public static double lastelevatorVelKi = 0;
    public static double lastelevatorVelKd = 0;
    public static double lastelevatorVelKf = 0;



    private double currentVoltage;
    private double currentAngle;
    private double currentAngleOffset = 0;

    private double lastVoltage = 0;
    private double lastAngle = 0;
    private double turretLastError = 0;

    private double actualAngle;
    private double greatestDiff = -0x80000000;

    private boolean firstLoop = true;

    private double turretIntegralSum = 0;
    private double turretTime = 0;

    // Autonomous Actions
    public class SpinLauncherAction implements Action {

        private boolean initialized = false;

        private double velocity;

        public SpinLauncherAction(double velocity) {
            this.velocity = velocity;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (!initialized) {
                launcherMotor1.setVelocity(velocity);
                launcherMotor2.setVelocity(velocity);
                initialized = true;
            }


            // Add some Debugging Helpers
            double measuredVelocity1 =  launcherMotor1.getVelocity();
            double measuredVelocity2 =  launcherMotor2.getVelocity();
            double measuredVelocityTotal = measuredVelocity1 + measuredVelocity2;
            // Logging
            if (measuredVelocity1 != 0.0 || measuredVelocity2 != 0.0) {
                //RobotLog.d("m1: %f m2: %f", measuredVelocity1, measuredVelocity2);
            }

            if (measuredVelocity1 == 0 && measuredVelocity2 > velocity/2 || measuredVelocity2 == 0 && measuredVelocity1 > velocity/2) {
                measuredVelocityTotal = (measuredVelocity1 + measuredVelocity2) * 2;
            }
            return measuredVelocityTotal < velocity * 2;
        }
    }

    public class SetLauncherPowerAction implements Action {

        private boolean initialized = false;

        public double power;

        public SetLauncherPowerAction(double power) {
            this.power = power;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (!initialized) {
                launcherMotor1.setPower(power);
                launcherMotor2.setPower(power);
                initialized = true;
            }
            return false;
        }
    }

    public class SetKickerPositionAction implements Action {

        private boolean initialized = false;

        public double position;

        public SetKickerPositionAction(double position) {
            this.position = position;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (!initialized) {
                kickerServo.setPosition(position);
                initialized = true;
            }

            return false;
        }
    }

    public class AprilTagAction implements Action {
        private boolean initialized = false;

        private ArtifactColor[] motifPattern;

        public AprilTagAction(int pipeline) {
            limelight.pipelineSwitch(pipeline);
        }


        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            ArtifactColor[] result = getMotifPattern(true);

            if (result != null) {
                motifPattern = result;
                return false;
            }
            return true;
        }

        public ArtifactColor[] getPattern() {
            return motifPattern;
        }
    }

    public Action getSpinLauncherAction(double velocity) {
        return new SequentialAction(
            new SpinLauncherAction(velocity));
    }

    public Action getWaitUntilVelocityAction(double velocity, double timeout) {
        return new RunTimeoutAction(
            new WaitUntilAction(() -> launcherMotor1.getVelocity() + launcherMotor2.getVelocity() == velocity),
            timeout
        );
    }

    public Action getRotateKickerAction(double position) {
        return new SequentialAction(
            new SetKickerPositionAction(position),
            new SleepAction(0.100) // TODO 125 / 300
        );
    }

    public Action getKickBallAction() {
        return getRotateKickerAction(POSITION_KICKER_SERVO_KICK_BALL);
    }

    public Action getResetKickerAction() {
        return getRotateKickerAction(POSITION_KICKER_SERVO_INIT);
    }

    public Action getSetLauncherPowerAction(double power) {
        return new SetLauncherPowerAction(power);
    }

    public Action getStopLauncherAction() {
        return getSetLauncherPowerAction(0);
    }

    public Action getAprilTagAction () {
        return new RunTimeoutAction(new AprilTagAction(7), 8);
    }

    public Launcher(HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        debugManager = new DebugManager(telemetry, "launcher");
        initializeLauncherDevices();
    }


    private void initElevatorMotor(){
        //give time to indexer to turn to outtake
        try {
            currentThread().sleep(500);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        //initialize device
        elevatorMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        elevatorMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        elevatorMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //run until magnetic limit switch detects
        elevatorMotor.setPower(.5);
        ElapsedTime motorTimer = new ElapsedTime();
        //limit switch detector?????????????
        while(motorTimer.milliseconds() < 1010){
            if(elevatorLimitSwitch.isPressed()){
                elevatorMotor.setPower(0);
                elevatorMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                elevatorMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                elevatorMotor.setPower(1);
                elevatorMotor.setTargetPosition(0);
            }
            if(motorTimer.milliseconds() > 1000 && !elevatorLimitSwitch.isPressed()){
                elevatorMotor.setPower(0);
                throw new Error("The elevator motor has not reached its target position in the allotted timeframe. Please check if it is stuck.");
            }
        }

    }

    
//    private void turnServoToLimitSwitch(){
//        launcherServo.setPower(.01);
//        ElapsedTime turretServoInitTimer = new ElapsedTime();
//
//        while(turretServoInitTimer.milliseconds() < 50000){
//            if(getTurretDegrees() == 76.33 && turretLimitSwitch.isPressed()){
//                launcherServo.setPower(0);
//            }
//            else if(getTurretDegrees() == 76.33 && !turretLimitSwitch.isPressed()){
//                turretServo.setPower(-.01);
//            }
//        }
//    }

    public void initializeLauncherDevices () {
        launcherMotor1 = hardwareMap.get(DcMotorEx.class, "launcherMotor1");
        launcherMotor2 = hardwareMap.get(DcMotorEx.class, "launcherMotor2");

        launcherMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        launcherMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        launcherMotor1.setDirection(DcMotorSimple.Direction.REVERSE);
        launcherMotor2.setDirection(DcMotorSimple.Direction.FORWARD);
        launcherMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcherMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        PIDFCoefficients pidfNew = new PIDFCoefficients(shootKp, shootKi, shootKd, shootKf);
        launcherMotor1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfNew);
        launcherMotor2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfNew);

        kickerServo = hardwareMap.get(Servo.class, "kickerServo");
        launcherServo = hardwareMap.get(CRServo.class, "turretServo");
        launcherAnalogInput = hardwareMap.get(AnalogInput.class, "turretAnalog");
        hoodServo = hardwareMap.get(Servo.class, "hoodServo");
        turretLimitSwitch = hardwareMap.get(TouchSensor.class, "magneticLimitTurret");

        kickerPosition = POSITION_KICKER_SERVO_INIT;
        kickerServo.setPosition(POSITION_KICKER_SERVO_INIT);

        hoodPosition = POSITION_HOOD_SERVO_HIGH;
        hoodServo.setPosition(hoodPosition);

        // initialized turret variables
        currentVoltage = launcherAnalogInput.getVoltage();
        currentAngle = currentVoltage / 3.3 * 360;
        // set the offset to the value from Autonomous
        if (RobotStaticValuesClass.autoCompleted) {
            currentAngleOffset = RobotStaticValuesClass.turretAngleOffset;
        }

        //0.7 voltage is the position of the limit switch

        //TODO turnServoToLimitSwitch();

        //348.55 is max with no power
        // 378.33 with power
        //369 with -power

        // min is 0
        // 38.84 with power
        // 25 with - power

        actualAngle = currentAngle + currentAngleOffset;

        debugManager.launcher("Servo Voltage", "%.2f", currentVoltage);
        debugManager.launcher("Servo Angle Raw", "%.2f", currentAngle);
        debugManager.launcher("Last Servo Voltage", "%.2f", lastVoltage);
        debugManager.launcher("Last Servo Angle Raw", "%.2f", lastAngle);
        debugManager.launcher("Actual Servo Angle", "%.2f", actualAngle);

        elevatorMotor = hardwareMap.get(DcMotorEx.class, "elevatorMotor");
        elevatorLimitSwitch = hardwareMap.get(TouchSensor.class, "magneticLimitElevator");

        initElevatorMotor();

        //elevatorMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //elevatorMotor.setTargetPosition(0);
        //elevatorMotor.setPower(1);

        PIDFCoefficients elevatorPID=elevatorMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION);
        RobotLog.d("Position PID: %.2f, %.2f, %.2f, %.2f", elevatorPID.p, elevatorPID.i, elevatorPID.d, elevatorPID.f);

        PIDFCoefficients elevatorVelPID=elevatorMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        RobotLog.d("Velocity PID: %.2f, %.2f, %.2f, %.2f", elevatorVelPID.p, elevatorVelPID.i, elevatorVelPID.d, elevatorVelPID.f);

        // Set positional constants
        elevatorKp = elevatorPID.p;
        elevatorKi = elevatorPID.i;
        elevatorKd = elevatorPID.d;
        // elevatorKf = elevatorPID.f;

        // set velocity constants
        elevatorVelKp = elevatorVelPID.p;
        elevatorVelKi = elevatorVelPID.i;
        elevatorVelKd = elevatorVelPID.d;
        elevatorVelKf = elevatorVelPID.f;


        lastAngle = currentAngle;
        lastVoltage = currentVoltage;

        shootingTable = new InterpolationTable(InterpolationTable.ExtrapolationMode.LINEAR);

        // key = distance from goal,
        // data1 = motor power,
        // data2 = shooter hood servo position
//        shootingTable.add(35,1080,1.0);
//        shootingTable.add(40.03,1100,1.0);
//        shootingTable.add(45.29,1120,0.73);
//        shootingTable.add(50.03,1140,0.72);
//        shootingTable.add(55.25,1160,0.66);
//        shootingTable.add(60.17,1180,0.50);
//        shootingTable.add(64.89,1200,0.32);
//        shootingTable.add(69.98,1230,0.30);
//        shootingTable.add(74.92,1250,0.275);
//        shootingTable.add(80.08,1280,0.24);
//        shootingTable.add(90,1320,0.13);
//        shootingTable.add(95.03,1350,0.11);
//        shootingTable.add(100.17,1370,0.10);
//        shootingTable.add(105.16,1380,0.10);
//        shootingTable.add(110.08,1420,0.10);
//        shootingTable.add(115.05,1480,0.08);
//        shootingTable.add(120.25,1500,0.03);
//        shootingTable.add(125.20,1520,0.025);
//        shootingTable.add(130.20,1550,0.02);
//        shootingTable.add(135.20,1580,0.01);
//        shootingTable.add(140.09,1600,0.00);
//        shootingTable.add(145.39,1640,0.00);
//        shootingTable.add(149.96,1660,0.00);
        shootingTable.add(35,1080,1.0);
        shootingTable.add(40.03,1100,1.0);
        shootingTable.add(45.29,1120,0.73);
        shootingTable.add(50.03,1140,0.72);
        shootingTable.add(55.25,1160,0.66);
        shootingTable.add(60.17,1180,0.50);
        shootingTable.add(64.89,1200,0.32);
        shootingTable.add(69.98,1230,0.30);
        shootingTable.add(74.92,1250,0.275);
        shootingTable.add(80.08,1280,0.24);
        shootingTable.add(90,1320,0.13);
        shootingTable.add(95.03,1350,0.11);
        shootingTable.add(100.17,1370,0.10);
        shootingTable.add(105.16,1380,0.10);
        shootingTable.add(110.08,1420,0.10);
        shootingTable.add(115.05,1480,0.08);
        shootingTable.add(120.25,1500,0.03);
        shootingTable.add(125.20,1520,0.025);
        shootingTable.add(130.20,1540,0.02);
        shootingTable.add(135.20,1560,0.01);
        shootingTable.add(140.09,1580,0.00);
        shootingTable.add(145.39,1620,0.00);
        shootingTable.add(149.96,1640,0.00);



        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);

        // TODO faster poll speed for localization
        limelight.setPollRateHz(50);

        limelight.start();

        // Initialize the map with calibration points.
        // Distances in cm, velocities as motor power (0.0 to 1.0)
        // Example values:
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

    public ArtifactColor[] getMotifPattern(boolean isRed) {
        LLResult result = limelight.getLatestResult();
        if (result.isValid()) {
            List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
            if (fiducialResults != null) {
                if (fiducialResults.size() > 1) {
                    int[] ids = new int[2];

                    ids[0] = fiducialResults.get(0).getFiducialId();
                    ids[1] = fiducialResults.get(1).getFiducialId();

                    if (isRed) {
                        if (Arrays.stream(ids).anyMatch((int i) -> i == 21) && Arrays.stream(ids).anyMatch((int i) -> i == 22)) {
                            return new ArtifactColor[]{ArtifactColor.GREEN, ArtifactColor.PURPLE, ArtifactColor.PURPLE};
                        }
                        else if (Arrays.stream(ids).anyMatch((int i) -> i == 23) && Arrays.stream(ids).anyMatch((int i) -> i == 22)) {
                            return new ArtifactColor[]{ArtifactColor.PURPLE, ArtifactColor.GREEN, ArtifactColor.PURPLE};
                        }
                        else if (Arrays.stream(ids).anyMatch((int i) -> i == 21) && Arrays.stream(ids).anyMatch((int i) -> i == 23)) {
                            return new ArtifactColor[]{ArtifactColor.PURPLE, ArtifactColor.PURPLE, ArtifactColor.GREEN};
                        }
                    }
                    else {
                        if (Arrays.stream(ids).anyMatch((int i) -> i == 21) && Arrays.stream(ids).anyMatch((int i) -> i == 22)) {
                            return new ArtifactColor[]{ArtifactColor.PURPLE, ArtifactColor.GREEN, ArtifactColor.PURPLE};
                        }
                        else if (Arrays.stream(ids).anyMatch((int i) -> i == 23) && Arrays.stream(ids).anyMatch((int i) -> i == 22)) {
                            return new ArtifactColor[]{ArtifactColor.PURPLE, ArtifactColor.PURPLE, ArtifactColor.GREEN};
                        }
                        else if (Arrays.stream(ids).anyMatch((int i) -> i == 21) && Arrays.stream(ids).anyMatch((int i) -> i == 23)) {
                            return new ArtifactColor[]{ArtifactColor.GREEN, ArtifactColor.PURPLE, ArtifactColor.PURPLE};
                        }
                    }
                }
                else {
                    if (fiducialResults.get(0).getFiducialId() == 21) {
                        return new ArtifactColor[]{ArtifactColor.GREEN, ArtifactColor.PURPLE, ArtifactColor.PURPLE};
                    } else if (fiducialResults.get(0).getFiducialId() == 22) {
                        return new ArtifactColor[]{ArtifactColor.PURPLE, ArtifactColor.GREEN, ArtifactColor.PURPLE};
                    } else if (fiducialResults.get(0).getFiducialId() == 23) {
                        return new ArtifactColor[]{ArtifactColor.PURPLE, ArtifactColor.PURPLE, ArtifactColor.GREEN};
                    }
                }
            }
        }
        return null;
    }

    private boolean launcherActive = false;

    @Deprecated
    public void kickBall() {
        RobotLog.d("kickball");
        if (isLauncherActive()) {
            kickerServo.setPosition(POSITION_KICKER_SERVO_KICK_BALL);
        }
    }
    @Deprecated
    public void resetKicker() {
        RobotLog.d("reset kicker");
        kickerServo.setPosition(POSITION_KICKER_SERVO_INIT);
    }

    boolean runElevator = false;
    double elevatorTarget = 0;

    double ticksPerElevate = 78.4;


    public void elevateBall() {
        runElevator = true;

        //  TODO 6000 rpm motor (not enough torque) 28ppr => (*2.8 rev) => 78.4 tick for a single flapper 1-shot
        //  TODO 1150 motor installed  145.1 ppr => (*2.8 rev) => 406.28 revolution for a 2 flapper 2-shot.
        elevatorTarget += 406.28;   // 78.4 * 4;
        elevatorMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void updateElevator() {
        if (runElevator) {
            elevatorMotor.setPower(1);
            if (elevatorMotor.getCurrentPosition() >= elevatorTarget) {
                runElevator = false;
                elevatorMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
        }
        else {
            elevatorMotor.setTargetPosition((int) Math.round(elevatorTarget));
        }
        RobotLog.d ("Elevator: pos: %d, vel: %.2f, launcher: %.2f, target: %.2f", elevatorMotor.getCurrentPosition(), elevatorMotor.getVelocity(), launcherMotor1.getVelocity(), launcherVelocity);
    }

    public boolean elevatorRunning() {
        return runElevator;
    }


    public double getKickerPosition() {
        return (double) Math.round(kickerServo.getPosition()*100)/100;
    }

    public void toggleLauncher() {
        debugManager.launcher("toggleLauncher", "%.2f", launcherMotor1.getPower());
       if (launcherMotor1.getPower() == 0) {
           startLauncher();
       }
       else {
           stopLauncher();
       }
    }

//    public void toggleLauncherManualFar() {
//        if (launcherMotor1.getPower() == 0) {
//            startLauncherManualFar();
//        }
//        else {
//            stopLauncher();
//        }
//    }

    /***********************************
     ***********ROTATOR SERVO***********
     ***********************************/
    public double getRawRotatorServoPower(){
        double output = turretServo.getPower();
        return output;
    }

    public double getRawRotatorServoPosition(){return  RSFeedback.getVoltage() * (180/3.3);}

    public double getRotatorServoPosition(){
        if(currentQuadrant == QuadrantRotatorServo.NEGATIVE){
            return getRawRotatorServoPosition();
        }
        if(currentQuadrant == QuadrantRotatorServo.POSITIVE){
            return getRawRotatorServoPosition() + 180;
        }
        if(currentQuadrant == QuadrantRotatorServo.ZERO){
            return 180;
        }
        else{
            return 0;
        }
    }
    public void startLauncher() {
        debugManager.launcher("startLauncher","%.2f",launcherVelocity);
        //launcherVelocity = shootingTable.getData1(shootingDistance);
        setLauncherVelocity(launcherVelocity);
        launcherActive = true;
    }

    public double convertFromDegreesToVoltage(double degrees){
        return degrees * (3.3/180);
    }


    public double getRotatorServoVoltage(){
        return RSFeedback.getVoltage();
    }

    public void setRotatorServoPower(double power){
        turretServo.setPower(power);

        if(power > 0 && getRawRotatorServoPosition() >= 180){
            currentQuadrant = QuadrantRotatorServo.POSITIVE;
        }
        else if(power < 0 && (lastServoPosition - getRawRotatorServoPosition()) < -3.5){
            currentQuadrant = QuadrantRotatorServo.NEGATIVE;
        }
        else if (getRawRotatorServoPosition() == 0){
            currentQuadrant = QuadrantRotatorServo.ZERO;
        }

        lastServoPosition = getRawRotatorServoPosition();
    }

    public double error() {return  lastServoPosition - getRawRotatorServoPosition();}

    public QuadrantRotatorServo getCurrentQuadrantOfRotatorServo(){return currentQuadrant;}

    public double targetRotatorPositionPIDControl(double targetDegrees, ElapsedTime timer){
        double error = (targetDegrees) - (getRotatorServoPosition());
        rotateIntegralSum += error*timer.seconds();
        double derivative = (error - lastError) / timer.seconds();
        double output = (rotateKp * error) + (rotateKi * rotateIntegralSum) + (rotateKd * derivative);
        lastError = error;
        timer.reset();

        return Range.clip(output, -1, 1);
    }

    /******************************
     **********HOOD SERVO**********
     ******************************/
    public void setHoodServoPosition(double position){
        hoodServo.setPosition(position);
    }

    public double getHoodServoPosition(){return hoodServo.getPosition();}

    public void setHoodServoDirection(double direction){
        if(direction > 0.05)
            hoodServo.setPosition(hoodServo.getPosition() - 0.05);
        else if (direction < -0.05) {
            hoodServo.setPosition(hoodServo.getPosition() + 0.05);
        }
    }

    public void changeHood(double change) {
        hoodPosition += change;

        if (hoodPosition > 1.0) {
            hoodPosition = 1.0;
        }
        else if (hoodPosition < 0.0) {
            hoodPosition = 0.0;
        }

        hoodServo.setPosition(hoodPosition);
    }

    /*****************************
     *******LAUNCHER MOTOR********
     *****************************/

    public void increaseLauncherMotorPower(){
        launcherMotor1.setPower(launcherMotor1.getPower() + 0.25);
        launcherMotor2.setPower(launcherMotor2.getPower() + 0.25);
    }
    public void decreaseLauncherMotorPower(){
        launcherMotor1.setPower(launcherMotor1.getPower() - 0.25);
        launcherMotor2.setPower(launcherMotor2.getPower() - 0.25);
    }

    public double getLauncherMotorPower(){return launcherMotor1.getPower();}

    public void reduceLauncherPower() {
        if (launchPower >= 0.1) {
            launchPower -= 0.1;
            launcherActive = true;
        }
        else{
            launchPower = 0;
            launcherActive = false;
        }
        setLauncherPower(launchPower);
    }

    public void stopLauncher() {
        launchPower = 0;
        setLauncherPower(launchPower);
        launcherActive = false;
    }

    public void setLauncherPower(double power) {
        launcherMotor2.setPower(power);
        launcherMotor1.setPower(power);
    }

    public double getLaunchPower(){
        return launcherMotor1.getPower();
    }

    public double getLauncherVelocity() {
        currentVelocity = launcherMotor1.getVelocity();
        return currentVelocity;
    }

    public double getLauncherTargetVelocity(){
        return launcherVelocity;
    }

    public double getLauncherVelocity2() {
        double currentVelocity2 = launcherMotor2.getVelocity();
        return currentVelocity2;
    }

    public Boolean isLauncherActive(){
        return launcherActive;
    }

    public void setLauncherVelocity(double velocity) {
        launcherMotor2.setVelocity(launcherVelocity);
        launcherMotor1.setVelocity(launcherVelocity);
    }

    public void setAutoVelocity(double velocity) {
        launcherMotor2.setVelocity(velocity);
        launcherMotor1.setVelocity(velocity);
        launcherActive = true;
    }


    public void changeLauncherVelocity(double change) {
        launcherVelocity += change;

        if (launcherVelocity > LAUNCH_VELOCITY_FULL) {
            launcherVelocity = LAUNCH_VELOCITY_FULL;
        }
        else if (launcherVelocity < 0.0) {
            launcherVelocity = 0.0;
        }

        setLauncherVelocity(launcherVelocity);
        launcherActive = (launcherVelocity != 0.0);
    }

    public void changeKicker(double change) {
        kickerPosition += change;

        if (kickerPosition > 1.0) {
            kickerPosition = 1.0;
        }
        else if (kickerPosition < 0.0) {
            kickerPosition = 0.0;
        }

        //kickerServo.setPosition(kickerPosition);
    }

    public double getKickerServoPosition() {
        return kickerServo.getPosition();
    }

    public void setKickerServoPosition(double position) {
        kickerServo.setPosition(position);
    }


    //For launch motor coefficients testing only
    public void setLaunchMotorPIDFCoefficients() {
        // Change coefficients using methods included with DcMotorEx class.
        PIDFCoefficients pidfNew = new PIDFCoefficients(shootKp, shootKi, shootKd, shootKf);
        launcherMotor1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfNew);
        launcherMotor2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfNew);
    }

    public PIDFCoefficients getLauncherMotorPIDFCoefficients() {
        return launcherMotor1.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void autoSetLauncherToGoal(Pose pose, AutoTaskMaker.Team team) {
        Pose ftcPose = InvertedFTCCoordinates.INSTANCE.convertFromPedro(pose);

        double goalX;
        double goalY;

        if (team == AutoTaskMaker.Team.RED) {
            goalX = -64;
            goalY = 64;
        }
        else {
            goalX = -64;
            goalY = -64;
        }

        double dist = Math.sqrt(Math.pow(goalX - ftcPose.getX(), 2) + Math.pow(goalY - ftcPose.getY(), 2));
        double targetDeg = Math.toDegrees(Math.atan2(goalX - ftcPose.getX(), goalY - ftcPose.getY()) - ftcPose.getHeading());

        targetDeg = ((targetDeg + 180) % 360) - 180;
        if (targetDeg < 0) {
            targetDeg += 360;
        }

        telemetry.addData("Dist", dist);
        telemetry.addData("Degrees", targetDeg);


        setLauncherVelocity(getDistanceVelocity(dist));
        autoUpdateTurretPID(targetDeg);
        setHoodServoPosition(getDistanceHoodPos(dist));
    }

    // Turret Methods
    public void setTurretRelativeAngle(double relativeTargetAngle){

        double turretLastTime = turretTime;
        turretTime = System.nanoTime() / 1000000000.0;
        double dt = turretTime - turretLastTime;

        double turretTarget = relativeTargetAngle * 2;
        // Find the voltage returned and the angle of the servo

        currentVoltage = launcherAnalogInput.getVoltage();
        //currentAngle = currentVoltage / 3.3 * 360;
        // Voltage only outputs angles between 2.5 deg and 355 deg
        currentAngle = ((currentVoltage / 3.3 * 360) - 2.5) / 352.5 * 360; // max 355, min 2.5

        // Find out whether the angle looped around

        double diff = Math.abs(currentAngle - lastAngle);

        if (!firstLoop) {
            if (currentAngle > 180 && lastAngle < 180 && diff > 100) {
                currentAngleOffset -= 360;
            }

            if (currentAngle < 180 && lastAngle > 180 && diff > 100) {
                currentAngleOffset += 360;
            }
        }

        actualAngle = currentAngle + currentAngleOffset;

        greatestDiff = Math.max(greatestDiff, diff);

        // Set Servo power
        double turretPower = updateTurretPID(turretTarget, actualAngle, dt);
        launcherServo.setPower(turretPower);

        // Set hood Servo position
        hoodPosition = shootingTable.getData2(shootingDistance);
        hoodServo.setPosition(hoodPosition);

        // Set launcher power
        launcherVelocity = shootingTable.getData1(shootingDistance);
        if (isLauncherActive())
            setLauncherVelocity(launcherVelocity);

        // Add telemetry data for debugging

        debugManager.addData("Turret Power", "%.2f", turretPower);
        debugManager.addData("Servo Voltage", "%.2f", currentVoltage);
        debugManager.addData("Servo Angle Raw", "%.2f", currentAngle);
        debugManager.addData("Turret target angle", "%.2f", turretTarget);
        debugManager.addData("Last Servo Voltage", "%.2f", lastVoltage);
        debugManager.addData("Last Servo Angle Raw", "%.2f", lastAngle);
        debugManager.addData("Difference", "%.2f", diff);
        debugManager.addData("Angle Offset", "%.2f", currentAngleOffset);
        debugManager.addData("Actual Servo Angle", "%.2f", actualAngle);
        debugManager.addData("launcherVelocity", "%.2f", launcherVelocity);
        debugManager.addData("launcherVelocity from motor1", "%.2f", launcherMotor1.getVelocity());
        debugManager.addData("launcherVelocity from motor2", "%.2f", launcherMotor2.getVelocity());
        debugManager.addData("hood position", "%.2f", hoodPosition);


        // Logging
        // RobotLog.d("Power: %.2f, Servo Angle: %.2f, Last Servo Angle: %.2f, Difference: %.2f, Angle Offset: %.2f, Actual Servo Angle: %.2f, target angle: %.2f", turretPower, currentAngle, lastAngle, diff, currentAngleOffset, actualAngle, turretTarget);
        // Set last variables for next loop

        lastAngle = currentAngle;
        lastVoltage = currentVoltage;

        firstLoop = false;
    }

    public double getTurretDegrees() {
        return actualAngle / 2;
    }

    public void autoUpdateTurretPID (double target) {

        double turretLastTime = turretTime;
        turretTime = System.nanoTime() / 1000000000.0;
        double dt = turretTime - turretLastTime;


        double turretTarget = target * 2;
        // Find the voltage returned and the angle of the servo

        currentVoltage = launcherAnalogInput.getVoltage();
        currentAngle = currentVoltage / 3.3 * 360;

        // Find out whether the angle looped around

        double diff = Math.abs(currentAngle - lastAngle);

        if (!firstLoop) {
            if (currentAngle > 180 && lastAngle < 180 && diff > 100) {
                currentAngleOffset -= 360;
            }

            if (currentAngle < 180 && lastAngle > 180 && diff > 100) {
                currentAngleOffset += 360;
            }
        }

        actualAngle = currentAngle + currentAngleOffset;

        // Set Servo power
        double turretPower = updateTurretPID(turretTarget, actualAngle, dt);
        launcherServo.setPower(turretPower);

        // Set last variables for next loop
        lastAngle = currentAngle;
        lastVoltage = currentVoltage;

        firstLoop = false;
    }

    public double updateTurretPID(double target, double current, double dt) {

        double error = target - current;

        if (Math.abs(error) < 1.5) {
            return 0;
        }

        turretIntegralSum += error * dt;

        double derivative = (error - turretLastError) / dt;
        turretLastError = error;

        double turretPower = Math.max(Math.min((error * turretkP) + (turretIntegralSum * turretkI) + (derivative * turretkD) + (turretkF * Math.signum(error)), 1), -1);

//        debugManager.launcher("turret target angle", "%.2f", target);
//        debugManager.launcher("turret current angle", "%.2f", current);
//        debugManager.launcher("turret Error", "%.2f", error);
//        debugManager.launcher("turret Integral", "%.2f", turretIntegralSum);
//        debugManager.launcher("turret Derivative", "%.2f", derivative);
//        debugManager.launcher("turret Power", "%.2f", turretPower);

        return turretPower;
    }

    public void setShootingDistance(double distanceToGoal) {
        shootingDistance = distanceToGoal;


    }

    public double getDistanceVelocity(double dist) {
        return shootingTable.getData1(dist);
    }

    public double getDistanceHoodPos(double dist) {
        return shootingTable.getData2(dist);
    }

    //For elevator motor coefficients testing only
    // TODO REMOVE FOR COMP
    public void setLiftMotorPIDFCoefficients() {
        // Change coefficients using methods included with DcMotorEx class.

        if (elevatorKp != lastelevatorKp
            || elevatorKi != lastelevatorKi
            || elevatorKd != lastelevatorKd
            || elevatorVelKp != lastelevatorVelKp
            || elevatorVelKi != lastelevatorVelKi
            || elevatorVelKd != lastelevatorVelKd
            || elevatorVelKf != lastelevatorVelKf)
        {
            lastelevatorKp = elevatorKp;
            lastelevatorKi = elevatorKi;
            lastelevatorKd = elevatorKd;

            lastelevatorVelKp = elevatorVelKp;
            lastelevatorVelKi = elevatorVelKi;
            lastelevatorVelKd = elevatorVelKd;
            lastelevatorVelKf = elevatorVelKf;



            // Positional
            // Be sure not to use PIDF for Positional Constants or it will generate runtime error



            PIDCoefficients pidNew = new PIDCoefficients(elevatorKp, elevatorKi, elevatorKd);
            elevatorMotor.setPIDCoefficients(DcMotor.RunMode.RUN_TO_POSITION, pidNew);


            // Velocity Constnats
            PIDFCoefficients pidfNew = new PIDFCoefficients(elevatorVelKp, elevatorVelKi, elevatorVelKd, elevatorVelKf);
            elevatorMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfNew);


            // Print one more time
            PIDFCoefficients elevatorPID=elevatorMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION);
            RobotLog.d("Position PID Upd: %.2f, %.2f, %.2f, %.2f", elevatorPID.p, elevatorPID.i, elevatorPID.d, elevatorPID.f);

            PIDFCoefficients elevatorVelPID=elevatorMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
            RobotLog.d("Velocity PID Upd: %.2f, %.2f, %.2f, %.2f", elevatorVelPID.p, elevatorVelPID.i, elevatorVelPID.d, elevatorVelPID.f);

        }

    }


}