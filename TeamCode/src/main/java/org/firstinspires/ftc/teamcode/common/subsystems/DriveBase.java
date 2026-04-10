package org.firstinspires.ftc.teamcode.common.subsystems;

import static android.os.SystemClock.sleep;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.common.Robot;
import org.firstinspires.ftc.teamcode.common.util.DebugManager;

import java.util.EnumMap;
import java.util.List;

public class DriveBase {

    HardwareMap hardwareMap;
    Telemetry telemetry;
    private final DebugManager debugManager;

    private DcMotor frontLeftMotor = null;
    private DcMotor frontRightMotor = null;
    private DcMotor backLeftMotor = null;
    private DcMotor backRightMotor = null;
    private Servo leftKickStand = null;
    private Servo rightKickStand = null;
    private Servo kickStandLight = null;

    // public IMU imu;
    // public double internalIMUHeadingOffset;

    // TODO adding poses for LL<->Pinpoint Sync
    public Pose3D botpose_mt2 ;
    public Pose3D botpose;
    private ElapsedTime PinpointTimer = new ElapsedTime();
    private ElapsedTime PinpointTimer2= new ElapsedTime();


    public GoBildaPinpointDriver pinpoint;
    // private GoBildaPinpointDriver.DeviceStatus lastStatus;

    private Pose2D pos;
    private boolean isRedSide;

    public boolean kickStandIsSet = false;
    private Integer parkingState =0;

    public ElapsedTime timeKickStand = new ElapsedTime();
    public ElapsedTime areWeThereYet = new ElapsedTime();


    public DriveBase(HardwareMap hardwareMap, Telemetry telemetry, boolean isRed) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        debugManager = new DebugManager(telemetry, "DRIVE");
        isRedSide = isRed;

        initializeDriveBaseDevices(isRed);
    }

    public void initializeDriveBaseDevices(boolean isRed) {
        frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        frontRightMotor = hardwareMap.get(DcMotor.class, "frontRightMotor");
        backLeftMotor = hardwareMap.get(DcMotor.class, "backLeftMotor");
        backRightMotor = hardwareMap.get(DcMotor.class, "backRightMotor");
        rightKickStand = hardwareMap.get(Servo.class, "rightKickStand");
        leftKickStand = hardwareMap.get(Servo.class, "leftKickStand");
        // kickStandLight = hardwareMap.get(Servo.class, "kickStandLight");

        // TODO bring back our old Friend MrInternalIMU
        // imu = hardwareMap.get(IMU.class, "imu");

        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // initialize the kick stand servos
        rightKickStand.setPosition(0.5);
        leftKickStand.setPosition(0.5);

        // ground lights OFF
        //kickStandLight.setPosition(0.0);

        // Get a reference to the sensor
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        // Configure the sensor
        configurePinpoint();
        // TODO will defer to the single pinpoint.update below.
        // pinpoint.update();

        // Configure onboard IMU (for localization error correction)
//        imu.initialize(new IMU.Parameters(
//                new RevHubOrientationOnRobot(
//                    RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
//                    RevHubOrientationOnRobot.UsbFacingDirection.UP
//                    )
//                )
//        );
//        imu.resetYaw();


//        //read the pose value from autonomous or initialized it at start up location
//        Pose2D startPose2D;
//        // if the auto completed, use the value from end of auto
//        if (RobotStaticValuesClass.autoCompleted) {
//            startPose2D = RobotStaticValuesClass.savedPose;
//            // TODO temp fix invalidate the static data (next run must be start at zero)
//            RobotStaticValuesClass.autoCompleted = false;
//        }
//        else if (isRed) {
//            //initialized red far position is x=(71in - 7.75in), y=23.5/2, -pi.
//            startPose2D = new Pose2D(DistanceUnit.INCH, 63.25, 11.75, AngleUnit.RADIANS, -3.14);
//        }
//        else {
//            //initialized red far position is x=(71in - 7.75in), y=-23.5/2, -pi.
//            startPose2D = new Pose2D(DistanceUnit.INCH, 63.25, -11.75, AngleUnit.RADIANS, -3.14);
//        }
//
//        // Set the location of the robot - this should be the place you are starting the robot from
//        pinpoint.setPosition(startPose2D);
//        pinpoint.update();
//        debugManager.addData("DriveBase Pos x, y, heading", "%.2f, %.2f, %.2f",
//            startPose2D.getX(DistanceUnit.INCH),
//            startPose2D.getY(DistanceUnit.INCH),
//            startPose2D.getHeading(AngleUnit.RADIANS));
//        debugManager.log("Pos x: %.2f, y: %2f, heading: %.2f",
//            startPose2D.getX(DistanceUnit.INCH),
//            startPose2D.getY(DistanceUnit.INCH),
//            startPose2D.getHeading(AngleUnit.RADIANS));

    }

//    public void resetIMU_internal() { imu.resetYaw(); }
//    public IMU getIMU_internal() { return imu; }

    public void resetIMU() {
        resetPinpointIMU();
    }

    public void recalibratePinpoint() {
        pinpoint.recalibrateIMU();
    }

    public void setPinpointYScalar(double val) {
        pinpoint.setYawScalar(val);
    }

    public void resetPinpointIMU() {
        pinpoint.resetPosAndIMU();
    }

    public void configurePinpoint() {

        pinpoint.setOffsets(8, -3.25, DistanceUnit.INCH); //Tuned for 2026 Bot2 17Feb26 OK

        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
            GoBildaPinpointDriver.EncoderDirection.REVERSED);

        pinpoint.resetPosAndIMU();
        sleep(300);
        PinpointTimer2.reset();
    }

    public void updateSafePinpoint() {
//        telemetry.addData("X", pinpoint.getPosX(DistanceUnit.INCH));
//        telemetry.addData("Y", pinpoint.getPosY(DistanceUnit.INCH));

        try {
            pinpoint.update();
            // lastStatus = pinpoint.getDeviceStatus();
        } catch (Exception e) {
            RobotLog.e("PINPOINT ERROR DURING UPDATE", e.getMessage());
            RobotLog.d("pinpoint status: %s", pinpoint.getDeviceStatus());
        }

        if (PinpointTimer2.milliseconds() >= 250) { // throttle timer to keep from calling too many i2c.
            PinpointTimer2.reset();
            try {
                if (Math.abs(1.0 - pinpoint.getYawScalar()) > 0.1) {
                // TODO Pinpoint driver issue.  reset Yaw Scalar to good value and REload the lastgood heading
                pinpoint.setYawScalar(Robot.PINPOINT_B1_YAW_SCALAR); // initial Factory Yaw Scalar for Pinpoint from Bot1
                }
            } catch (Exception e) {
                RobotLog.e("PINPOINT ERROR during GET or SET YawScalar");
                RobotLog.d("pinpoint status: %s", pinpoint.getDeviceStatus());
            }
        }
        pos = pinpoint.getPosition();
        // RobotLog.d("pinpoint heading1 %.2f: yawScalar: %.8f", pinpoint.getHeading(AngleUnit.RADIANS), pinpoint.getYawScalar());


    }
    public void setMotorPowers(double x, double y, double rx, double speed, boolean fieldCentric) {

        // TODO pinpoint update once per loop (this function also checks for sane yawScalar)
        updateSafePinpoint();

        double heading = 0;
        double driverOffset;
        double adjustedHeading;
        if (fieldCentric) {
            heading = pos.getHeading(AngleUnit.RADIANS);
            // Apply 90-degree driver offset (in radians)
            if (isRedSide) {
                driverOffset = Math.toRadians(-90); // Change to -90 if opposite direction needed
            } else {
                driverOffset = Math.toRadians(90);
            }

            adjustedHeading = -(heading - driverOffset);
            //if (adjustedHeading > 180)  adjustedHeading -= 360;
            //if (adjustedHeading < -180) adjustedHeading += 360;
        } else {
            adjustedHeading = 180; // 31mar26 test was 0 (reversed controls)
        }

        double rotX = x * Math.cos(adjustedHeading) - y * Math.sin(adjustedHeading);
        double rotY = x * Math.sin(adjustedHeading) + y * Math.cos(adjustedHeading);

        // put strafing factors here
        rotX = rotX * 1;
        rotY = rotY * 1;

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);

        double frontLeftPower = ((rotY + rotX) * speed + rx) / denominator;
        double backLeftPower = ((rotY - rotX) * speed + rx) / denominator;
        double frontRightPower = ((rotY - rotX) * speed - rx) / denominator;
        double backRightPower = ((rotY + rotX) * speed - rx) / denominator;

        frontLeftMotor.setPower(frontLeftPower);
        backLeftMotor.setPower(backLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backRightMotor.setPower(backRightPower);

        debugManager.addData("DrvieBase", " Heading %.2f, PosX,Y %.2f,%.2f",
            heading,
            pos.getX(DistanceUnit.MM),
            pos.getY(DistanceUnit.MM));
        debugManager.addData("DrvieBase fieldCentric", "%s", fieldCentric);
        debugManager.addData("DrvieBase powers", "front left: %.2f, front right: %.2f, back left: %.2f, back right: %.2f",
            frontLeftPower * speed * 100,
            frontRightPower * speed * 100,
            backLeftPower * speed * 100,
            backRightPower * speed * 100);
        debugManager.log("Heading %.2f, PosX,Y %.2f,%.2f",
            heading,
            pos.getX(DistanceUnit.MM),
            pos.getY(DistanceUnit.MM));
        debugManager.log("fieldCentric", fieldCentric);
        debugManager.log("powers front left: %.2f, front right: %.2f, back left: %.2f, back right: %.2f",
            frontLeftPower * speed * 100,
            frontRightPower * speed * 100,
            backLeftPower * speed * 100,
            backRightPower * speed * 100);
    }

    public void setIndividualMotorPowers(double frontLeftPower, double frontRightPower, double backRightPower, double backLeftPower) {
        frontLeftMotor.setPower(frontLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backRightMotor.setPower(backRightPower);
        backLeftMotor.setPower(backLeftPower);
    }

    public void setKickStand() {
        areWeThereYet.startTime();
        rightKickStand.setPosition(0.81);
        leftKickStand.setPosition(0.19);
        kickStandIsSet = true;
        parkingState = 0;
        timeKickStand.reset();
    }

    public void resetKickStand() {
        rightKickStand.setPosition(0.5);
        leftKickStand.setPosition(0.5);
        kickStandIsSet = false;
        parkingState = 4;
    }
    
    public void spinFrontWheels() {
        switch (parkingState) {
            case 0: // INIT
                if (timeKickStand.milliseconds()<=750) {
                    // wait for a bit so we don't move
                }
                else {
                    parkingState = 1;
                    timeKickStand.reset();
                }
                break;
            case 1: // Run wheels forward
                if (timeKickStand.milliseconds() <= 600)  {
                    frontRightMotor.setPower(1);
                    frontLeftMotor.setPower(-1);
                    break;
                }
                else {
                    timeKickStand.reset();
                    parkingState = 2;
                    break;
                }
            case 2: // Reverse Wheels for 300ms
                if (timeKickStand.milliseconds() <= 300) {
                    frontRightMotor.setPower(-1);
                    frontLeftMotor.setPower(1);
                    break;
                }
                else {
                    timeKickStand.reset();
                    parkingState = 3;
                    break;
                }
            case 3: // Reverse for final 300ms
                if (timeKickStand.milliseconds()<=300) {
                    frontRightMotor.setPower(1);
                    frontLeftMotor.setPower(-1);
                    break;
                }
                else {
                    timeKickStand.reset();
                    parkingState = 4;
                }
            case 4: // stop motors
                frontRightMotor.setPower(0);
                frontLeftMotor.setPower(0);
                parkingState=5;
                break;
            case 5: // end state
                break;
        }
    }

    public void setKickStandLight() {
        kickStandLight.setPosition(1.0);
    }

    public void resetKickStandLight(){
        kickStandLight.setPosition(0.0);
    }

    public void adjustKickStandLight(double power){
        kickStandLight.setPosition(power);
    }

    public double getPinPointPosX() {
        return pinpoint.getPosX(DistanceUnit.INCH);
    }

    public double getPinPointPosY() {
        return pinpoint.getPosY(DistanceUnit.INCH);
    }

    public double getPinPointHeading() {
        return pinpoint.getHeading(AngleUnit.RADIANS);
    }

    public Pose2D getPinPointPose() { return pinpoint.getPosition();}


    // TODO LOCALIZATION STUFF
    public void setPinPointPose(Pose2D pose) {
        pinpoint.setPosition(pose);
    }

    // This method can be changed to use an internal IMU or Pinpoint by uncommenting lines
    // Returns the current Heading IN DEGREES
    public double getHeadingDegrees()
    {
        // INTERNAL IMU ( if using internal, need to init at same time as pinpoint and set startposeoffset)
        // double heading = robot.getDriveBase().getIMU_internal().getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        // double startPoseOffset = robot.getDriveBase().internalIMUHeadingOffset;
        // heading += startingPoseOffset;

        // PINPOINT IMU
        double heading = AngleUnit.RADIANS.toDegrees(getPinPointHeading()); // Convert to DEGREES
        // RobotLog.d("DRIFT IMU: heading:%.2f, offset:%.2f",heading,startPoseOffset);
        return heading;
    }

    public void correctHeadingFromVision(LLResult result, Robot robot)
    {
        if (result == null || !result.isValid()) return;
        List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();

        // Need 2+ tags for reliable independent heading fix
        if (tags == null || tags.size() < 2) return;

        botpose = result.getBotpose(); // MegaTag1 (needs 2 tags, but doesn't need IMU)
        if (botpose == null) return;

        double visionHeading = botpose.getOrientation().getYaw(AngleUnit.DEGREES); // DEGREES
        double imuHeading = getHeadingDegrees(); // DEGREES
        double error =  AngleUnit.normalizeDegrees(visionHeading - imuHeading);
        // RobotLog.d("DRIFT correctHeading: v:%.2f, i:%.2f, e:%.2f",visionHeading,imuHeading,error);

        // Only correct if the heading error is significant, but not insane (bad tag read)
        if (Math.abs(error)>1.0 && Math.abs(error) < 30)
        {
            // correctedHeadingOffset += error; // * 0.2;
            // RobotLog.d("DRIFT correctedHeadingOffset: %.2f",correctedHeadingOffset);
            double correctedHeadingDegs = robot.normalizeAngle(imuHeading + error);
            // RobotLog.d("DRIFT unormHeading: %.2f newHeading: %.2f",getHeading(),correctedHeadingDegs);
            // TODO Update Pinpoint with Adjusted MT1 Heading (only)
            if (robot.enableDriftCorrection) {
                pinpoint.setHeading(correctedHeadingDegs, AngleUnit.DEGREES);
            }
        }

    }

    public void correctPositionFromVision(LLResult result, Robot robot)
    {

        botpose_mt2 = result.getBotpose_MT2();
        if (botpose_mt2 == null) return;

        double lx = botpose_mt2.getPosition().toUnit(DistanceUnit.INCH).x;
        double ly = botpose_mt2.getPosition().toUnit(DistanceUnit.INCH).y;
        double lr = botpose_mt2.getOrientation().getYaw(AngleUnit.DEGREES);

        // TODO Check for Sane values and Drift then Update the PINPOINT with XY correction from MT2
        if (Math.round(lx) != 0 && Math.abs(Math.round(lx)) < 72 && Math.round(ly) != 0 && Math.abs(Math.round(ly)) < 72 && PinpointTimer.milliseconds() > 250) {
            double px = getPinPointPosX();
            double py = getPinPointPosY();
            // double pr = AngleUnit.RADIANS.toDegrees(getPinPointHeading()); // convert from radians to degrees
            // RobotLog.d("DRIFT L:%.2f,%.2f,%.2f P:%.2f,%.2f,%.2f", lx, ly, lr, px, py, pr);

            // Check for a Delta greater than 2in avg and update pinpoint
            if (Math.abs(lx - px) > 2 || Math.abs(ly - py) > 2 && PinpointTimer.milliseconds() > 250) {
                PinpointTimer.reset();
                // TODO avg the result
                px = (px + lx) / 2;
                py = (py + ly) / 2;
                //pr = correctedHeadingDegs;

                // TODO Update Pinpoint with Drifted X,Y
                if (robot.enableDriftCorrection) {
                    pinpoint.setPosX(px, DistanceUnit.INCH);
                    pinpoint.setPosY(py, DistanceUnit.INCH);
                    // setPinPointPose(new Pose2D(DistanceUnit.INCH, px, py, AngleUnit.DEGREES, pr));
//                RobotLog.d("DRIFTED Pn:%.2f,%.2f,%.2f Po:%.2f,%.2f,%.2f", px, py, pr,
//                        getPinPointPosX(),
//                        getPinPointPosY(),
//                        AngleUnit.RADIANS.toDegrees(getPinPointHeading()));
                }
            }
        }
    }


    public void resetPinpointTimer() {
        PinpointTimer.reset();
    }
}