package org.firstinspires.ftc.teamcode.common.subsystems;

import static android.os.SystemClock.sleep;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.common.Robot;
import org.firstinspires.ftc.teamcode.common.RobotStaticValuesClass;
import org.firstinspires.ftc.teamcode.common.util.DebugManager;

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

    // private IMU imu;

    GoBildaPinpointDriver pinpoint;
    private GoBildaPinpointDriver.DeviceStatus lastStatus;

    private Pose2D pos;
    private boolean isRedSide;


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

        //read the pose value from autonomous or initialized it at start up location
        Pose2D startPose2D;
        // if the auto completed, use the value from end of auto
        if (RobotStaticValuesClass.autoCompleted) {
            startPose2D = RobotStaticValuesClass.savedPose;
            // TODO temp fix invalidate the static data (next run must be start at zero)
            RobotStaticValuesClass.autoCompleted = false;
        }
        else if (isRed) {
            //initialized red far position is x=(71in - 8.8in), y=23.5/2, -pi.
            startPose2D = new Pose2D(DistanceUnit.INCH, 62.2, 11.75, AngleUnit.RADIANS, -3.14);
        }
        else {
            //initialized red far position is x=(71in - 8.8in), y=-23.5/2, -pi.
            startPose2D = new Pose2D(DistanceUnit.INCH, 62.2, -11.75, AngleUnit.RADIANS, -3.14);
        }

        // Set the location of the robot - this should be the place you are starting the robot from
        pinpoint.setPosition(startPose2D);
        pinpoint.update();
        debugManager.addData("DriveBase Pos x, y, heading", "%.2f, %.2f, %.2f",
            startPose2D.getX(DistanceUnit.INCH),
            startPose2D.getY(DistanceUnit.INCH),
            startPose2D.getHeading(AngleUnit.RADIANS));
        debugManager.log("Pos x: %.2f, y: %2f, heading: %.2f",
            startPose2D.getX(DistanceUnit.INCH),
            startPose2D.getY(DistanceUnit.INCH),
            startPose2D.getHeading(AngleUnit.RADIANS));

    }

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
    }

    public void updateSafePinpoint()
    {
//        telemetry.addData("X", pinpoint.getPosX(DistanceUnit.INCH));
//        telemetry.addData("y", pinpoint.getPosY(DistanceUnit.INCH));

        try {
            pinpoint.update();
            lastStatus = pinpoint.getDeviceStatus();
        }
        catch (Exception e) {
            RobotLog.e("PINPOINT ERROR DURING UPDATE", e.getMessage());
            RobotLog.d("pinpoint status: %s", pinpoint.getDeviceStatus());
        }

        if (Math.abs(1.0 - pinpoint.getYawScalar()) > 0.1 )
        {
            // TODO Pinpoint driver issue.  reset Yaw Scalar to good value and REload the lastgood heading
            pinpoint.setYawScalar(Robot.PINPOINT_B1_YAW_SCALAR); // initial Factory Yaw Scalar for Pinpoint from Bot1
        }
        pos = pinpoint.getPosition();
        RobotLog.d("pinpoint heading1 %.2f, yawScalar: %.8f, lastStatus: %s", pinpoint.getHeading(AngleUnit.RADIANS), pinpoint.getYawScalar(), lastStatus);


    }
    public void setMotorPowers(double x, double y, double rx, double speed, boolean fieldCentric) {

        // TODO pinpoint update moved out
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
            adjustedHeading = 0;
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
        rightKickStand.setPosition(0.81);
        leftKickStand.setPosition(0.19);
    }

    public void resetKickStand() {
        rightKickStand.setPosition(0.5);
        leftKickStand.setPosition(0.5);
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


}