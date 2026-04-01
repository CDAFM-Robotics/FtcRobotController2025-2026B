package org.firstinspires.ftc.teamcode.common.subsystems;
import static android.os.SystemClock.sleep;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.common.util.ArtifactColor;
import org.firstinspires.ftc.teamcode.common.util.DebugManager;
import org.firstinspires.ftc.teamcode.common.util.RunTimeoutAction;
import org.firstinspires.ftc.teamcode.common.util.WaitUntilAction;

import java.util.Arrays;
import java.util.LinkedList;
import java.util.Queue;

public class Indexer {

    HardwareMap hardwareMap;
    Telemetry telemetry;
    private final DebugManager debugManager;

    public Servo indexerServo = null;
    private ElapsedTime timeSinceTurnIndex = new ElapsedTime();
    private int nextEmptySlot;
    private int nextShootSlot;
    private double targetIndexerPosition;

    //after indexer HW change, this is the left color sensor on the intake
    public NormalizedColorSensor colorSensorIntakeL = null;
    public NormalizedColorSensor colorSensorIntakeR = null;
    //after indexer HW change, this is the left color sensor on the back left
    public NormalizedColorSensor colorSensorBackRL = null;
    public NormalizedColorSensor colorSensorBackRR = null;
    //after indexer HW change, this is the left color sensor on the back right
    public NormalizedColorSensor colorSensorBackLL = null;
    public NormalizedColorSensor colorSensorBackLR = null;

    private AnalogInput indexerServoVoltage = null;

    public ArtifactColor[] artifactColorArray = new ArtifactColor[]{ArtifactColor.NONE, ArtifactColor.NONE, ArtifactColor.NONE};

    // TODO +0.03 after indexer rebuild on 20Mar26
    public static final double POSITION_INDEXER_SERVO_SLOT_ZERO_OUTPUT = 0.210 ; // 0.209 last value 0.177 + 0.03; //0.176
    public static final double POSITION_INDEXER_SERVO_SLOT_ONE_OUTPUT = 0.973; // last value 0.965 0.934 + 0.03 ; //0.938
    public static final double POSITION_INDEXER_SERVO_SLOT_TWO_OUTPUT = 0.590; // last value 0.583 0.55955 + 0.03; //0.555
    public static final double POSITION_INDEXER_SERVO_SLOT_ZERO_INTAKE = 0.776; // 0.783 last value 0.757 + 0.03; //0.746
    public static final double POSITION_INDEXER_SERVO_SLOT_ONE_INTAKE = 0.393 ; // 0.403 last value 0.380 + 0.03 ; // 0.365
    public static final double POSITION_INDEXER_SERVO_SLOT_TWO_INTAKE = 0.017; // 0.03 last value 0.00; //0.0

    public static final double AXON_SERVO_VOLTAGE_OFFSET = 0.228;
    public static final double AXON_SERVO_VOLTAGE_SCALER = 0.1 / 0.2815;

    /*******************************************************
     * constructor and hardware initialization
     * @param hardwareMap
     * @param telemetry
     *******************************************************/
    public Indexer(HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        debugManager = new DebugManager(telemetry, "spindexer");
        initializeIndexerDevices();
    }

    public void initializeIndexerDevices() {
        indexerServo = hardwareMap.get(Servo.class, "indexerServo");
        indexerServoVoltage = hardwareMap.get(AnalogInput.class, "indexerAnalog");

        indexerServo.setDirection(Servo.Direction.FORWARD);
        indexerServo.scaleRange(0.0, 1.0);

        colorSensorIntakeL = hardwareMap.get(NormalizedColorSensor.class, "colorSensorIntakeLeftLeft");
        colorSensorIntakeR = hardwareMap.get(NormalizedColorSensor.class, "colorSensorIntakeLeftRight");
        colorSensorBackLL = hardwareMap.get(NormalizedColorSensor.class, "colorSensorIntakeRightLeft");
        colorSensorBackLR = hardwareMap.get(NormalizedColorSensor.class, "colorSensorIntakeRightRight");
        colorSensorBackRL = hardwareMap.get(NormalizedColorSensor.class, "colorSensorOutLeft");
        colorSensorBackRR = hardwareMap.get(NormalizedColorSensor.class, "colorSensorOutRight");

        colorSensorIntakeL.setGain(8);
        colorSensorIntakeR.setGain(8);
        colorSensorBackRL.setGain(8);
        colorSensorBackRR.setGain(8);
        colorSensorBackLL.setGain(8);
        colorSensorBackLR.setGain(8);

        // have to turn to intake to read the ball colors. DON'T REMOVE!
        turnToClosestIntake();
        sleep(650);
        updateColorAllSlots();

        timeSinceTurnIndex.reset();

        //move to outtake in order to let elevator motor run
        //moveToOuttake() moves the next shooting slot to outtake
        // should use positionForOuttake
        positionForOuttake();

        while (indexerFinishedTurning()) {}
    }

    /****************************************************
     * Autonomous Actions
     ****************************************************/
    public class RotateIndexerAction implements Action {

        private boolean initialized = false;

        public double position;

        public RotateIndexerAction(double position) {
            this.position = position;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (!initialized) {
                indexerServo.setPosition(position);
                initialized = true;
            }

            return !getIndexerServoAtPosition(position, 0.08);

        }
    }

    public Action getRotateIndexerAction(double position) {
        return new RotateIndexerAction(position);
    }

    public Action getGoToZeroBallOutputAction() {
        return getRotateIndexerAction(POSITION_INDEXER_SERVO_SLOT_ZERO_OUTPUT);
    }

    public Action getGoToOneBallOutputAction() {
        return getRotateIndexerAction(POSITION_INDEXER_SERVO_SLOT_ONE_OUTPUT);
    }

    public Action getGoToTwoBallOutputAction() {
        return getRotateIndexerAction(POSITION_INDEXER_SERVO_SLOT_TWO_OUTPUT);
    }

    public Action getGotoZeroBallIntakeAction() {
        return getRotateIndexerAction(POSITION_INDEXER_SERVO_SLOT_ZERO_INTAKE);
    }

    public Action getGoToOneBallIntakeAction() {
        return getRotateIndexerAction(POSITION_INDEXER_SERVO_SLOT_ONE_INTAKE);
    }

    public Action getGoToTwoBallIntakeAction() {
        return getRotateIndexerAction(POSITION_INDEXER_SERVO_SLOT_TWO_INTAKE);
    }


    public Action getWaitUntilBallInIndexerAction(double timeout) {
        return new RunTimeoutAction(
            new WaitUntilAction(
                () -> getPredictedColor(
                    colorSensorIntakeL.getNormalizedColors(),
                    colorSensorIntakeR.getNormalizedColors(),
                    ((DistanceSensor) colorSensorIntakeL).getDistance(DistanceUnit.CM),
                    ((DistanceSensor) colorSensorIntakeR).getDistance(DistanceUnit.CM)) != ArtifactColor.NONE),

            timeout
        );
    }

    public void autoFillColorArray() {
        artifactColorArray[0] = ArtifactColor.PURPLE;
        artifactColorArray[1] = ArtifactColor.PURPLE;
        artifactColorArray[2] = ArtifactColor.PURPLE;
    }

    /****************************************************
     * Class methods
     *****************************************************/
    public boolean getIndexerServoAtPosition(double position, double accuracy) {
        double indexerPosition = getAxonServoPosition();
        return Math.abs(indexerPosition - position) < accuracy;
    }

    public double getAxonServoPosition() {
        debugManager.log("getAxonServoPosition: %.3f", indexerServoVoltage.getVoltage());
        return (indexerServoVoltage.getVoltage() - AXON_SERVO_VOLTAGE_OFFSET) * AXON_SERVO_VOLTAGE_SCALER;
    }

    public double getIndexerPosition() {
        double position = indexerServo.getPosition();
        debugManager.spindexer("getIndexerPosition position", "%.6f", position);
        debugManager.log("position get index pos: %.6f", position);
        return (double) Math.round(position * 1000) / 1000.00;
    }

    public double getTargetIndexerPosition() {
        return indexerServo.getPosition();
    }

    public boolean indexerFinishedTurning() {
        debugManager.spindexer("indexerFinishedTurning start", targetIndexerPosition);
        //TODO: 0.02 is used to start with. Is 0.02 the best value to use here?
        return getIndexerServoAtPosition(targetIndexerPosition, 0.03);
    }

    public void rotateToPosition(double position) {
        debugManager.log("rotateToPosition %.3f", position);
        targetIndexerPosition = position;
        indexerServo.setPosition(position);
    }

    public ArtifactColor getPredictedColor(NormalizedRGBA sensor1RGBA, NormalizedRGBA sensor2RGBA, double sensor1Distance, double sensor2Distance) {

        ArtifactColor sensor1DetectedColor;
        debugManager.spindexer("sensor1Distance", "%.2f", sensor1Distance);
        debugManager.spindexer("sensor1Distance alpha, ", "%.2f", sensor1RGBA.alpha);
        debugManager.log("sensor1Distance %.2f", sensor1Distance);
        debugManager.log("sensor1RGBA.alpha %.2f", sensor1RGBA.alpha);
        debugManager.spindexer("sensor2Distance", "%.2f", sensor2Distance);
        debugManager.spindexer("sensor2Distance alpha, ", "%.2f", sensor2RGBA.alpha);
        debugManager.log("sensor2Distance %.2f", sensor2Distance);
        debugManager.log("sensor2RGBA.alpha %.2f", sensor2RGBA.alpha);

        if (sensor1Distance > 3.5) {
            sensor1DetectedColor = ArtifactColor.NONE;
        }
        else if (sensor1RGBA.blue > sensor1RGBA.green) {
            sensor1DetectedColor = ArtifactColor.PURPLE;
        }
        else {
            sensor1DetectedColor = ArtifactColor.GREEN;
        }

        ArtifactColor sensor2DetectedColor;

        if (sensor2Distance > 3.5) {
            sensor2DetectedColor = ArtifactColor.NONE;
        }
        else if (sensor2RGBA.blue > sensor2RGBA.green) {
            sensor2DetectedColor = ArtifactColor.PURPLE;
        }
        else {
            sensor2DetectedColor = ArtifactColor.GREEN;
        }

        if (sensor1DetectedColor == sensor2DetectedColor) {
            return sensor1DetectedColor;
        }
        else if (sensor2DetectedColor == ArtifactColor.NONE) {
            return sensor1DetectedColor;
        }
        else if (sensor1DetectedColor == ArtifactColor.NONE) {
            return sensor2DetectedColor;
        }
        else {
            return ArtifactColor.UNKNOWN;
        }
    }


    public void updateBallColorAtBackL(double position) {
        debugManager.spindexer("updateBallColorAtBackL() start", "%.2f", position);
        debugManager.log("Indexer: updateBallColorAtBackL() start");

        int i = 0;

        if (position == POSITION_INDEXER_SERVO_SLOT_ZERO_INTAKE) {
            i = 1;
        }
        else if (position == POSITION_INDEXER_SERVO_SLOT_ONE_INTAKE) {
            i = 2;
        }
        else if (position == POSITION_INDEXER_SERVO_SLOT_TWO_INTAKE) {
            i = 0;
        }
        else {
            telemetry.addLine("ERROR: updateBallColors");
        }

        artifactColorArray[i] = getPredictedColor(
            colorSensorBackLL.getNormalizedColors(),
            colorSensorBackLR.getNormalizedColors(),
            ((DistanceSensor) colorSensorBackLL).getDistance(DistanceUnit.CM),
            ((DistanceSensor) colorSensorBackLR).getDistance(DistanceUnit.CM));
        debugManager.spindexer("updateBallColorAtBackL index", "%d", i);
        debugManager.spindexer("updateBallColorAtBackL color1", "%s", artifactColorArray[i]);
        debugManager.log("updateBallColorAtBackL color1 %d %s", i, artifactColorArray[i]);

    }

    public void updateBallColorAtIntake(double position) {
        debugManager.spindexer("updateBallColorAtIntake() start", "%.2f", position);
        debugManager.log("Indexer: updateBallColorAtIntake() start");

        int i = 0;

        if (position == POSITION_INDEXER_SERVO_SLOT_ZERO_INTAKE) {
            i = 0;
        } else if (position == POSITION_INDEXER_SERVO_SLOT_ONE_INTAKE) {
            i = 1;
        } else if (position == POSITION_INDEXER_SERVO_SLOT_TWO_INTAKE) {
            i = 2;
        } else {
            telemetry.addLine("ERROR: updateBallColors");
        }

        artifactColorArray[i] = getPredictedColor(
            colorSensorIntakeL.getNormalizedColors(),
            colorSensorIntakeR.getNormalizedColors(),
            ((DistanceSensor) colorSensorIntakeL).getDistance(DistanceUnit.CM),
            ((DistanceSensor) colorSensorIntakeR).getDistance(DistanceUnit.CM));
        debugManager.spindexer("updateBallColorAtIntake index", "%d", i);
        debugManager.spindexer("updateBallColorAtIntake color1","%s",  artifactColorArray[i]);
        debugManager.log("updateBallColors color Intake %d %s", i, artifactColorArray[i]);

    }

    public void updateBallColorAtBackR(double position) {
        debugManager.spindexer("updateBallColorAtBackR() start", "%.2f", position);
        debugManager.log("Indexer: updateBallColorAtBackR() start");

        int i = 0;

        if (position == POSITION_INDEXER_SERVO_SLOT_ZERO_INTAKE) {
            i = 2;
        } else if (position == POSITION_INDEXER_SERVO_SLOT_ONE_INTAKE) {
            i = 0;
        } else if (position == POSITION_INDEXER_SERVO_SLOT_TWO_INTAKE) {
            i = 1;
        } else {
            telemetry.addLine("ERROR: updateBallColors");
        }

        artifactColorArray[i] = getPredictedColor(
            colorSensorBackRL.getNormalizedColors(),
            colorSensorBackRR.getNormalizedColors(),
            ((DistanceSensor) colorSensorBackRL).getDistance(DistanceUnit.CM),
            ((DistanceSensor) colorSensorBackRR).getDistance(DistanceUnit.CM));
        debugManager.spindexer("updateBallColorAtBackR index", "%d", i);
        debugManager.spindexer("updateBallColorAtBackR color1", "%s", artifactColorArray[i]);
        debugManager.log("updateBallColorAtBackR  %d %s", i, artifactColorArray[i]);
    }

    public void updateColorAllSlots(){
        double position = getIndexerPosition();
        debugManager.spindexer("updateColorAllSlots Indexer Position", "%.2f", position);
        updateBallColorAtIntake(position);
        updateBallColorAtBackR(position);
        updateBallColorAtBackL(position);
        debugManager.spindexer("updateBallColors Color 0", "%s", artifactColorArray[0]);
        debugManager.spindexer("updateBallColors Color 1", "%s", artifactColorArray[1]);
        debugManager.spindexer("updateBallColors Color 2", "%s", artifactColorArray[2]);
        debugManager.log("updateBallColors Color 0, %s", artifactColorArray[0]);
        debugManager.log("updateBallColors Color 1, %s", artifactColorArray[1]);
        debugManager.log("updateBallColors Color 2, %s", artifactColorArray[2]);


    }

    public boolean checkEmptySlot() {

        //Check for empty slot according to current position
        double position = getIndexerPosition();
        debugManager.spindexer("checkEmptySlot", "%.2f", position);
        debugManager.log("checkEmptySlot %.3f", position);

        if (position >= POSITION_INDEXER_SERVO_SLOT_TWO_OUTPUT) {
            if (artifactColorArray[0] == ArtifactColor.NONE) {
                nextEmptySlot = 0;
                debugManager.log("checkEmptySlot %d", nextEmptySlot);
                return true;
            } else if (artifactColorArray[1] == ArtifactColor.NONE) {
                nextEmptySlot = 1;
                debugManager.log("checkEmptySlot %d", nextEmptySlot);
                return true;
            } else if (artifactColorArray[2] == ArtifactColor.NONE) {
                nextEmptySlot = 2;
                debugManager.log("checkEmptySlot %d", nextEmptySlot);
                return true;
            }
        } else if (position >= POSITION_INDEXER_SERVO_SLOT_ONE_INTAKE) {
            if (artifactColorArray[1] == ArtifactColor.NONE) {
                nextEmptySlot = 1;
                debugManager.log("checkEmptySlot %d", nextEmptySlot);
                return true;
            } else if (artifactColorArray[2] == ArtifactColor.NONE) {
                nextEmptySlot = 2;
                debugManager.log("checkEmptySlot %d", nextEmptySlot);
                return true;
            } else if (artifactColorArray[0] == ArtifactColor.NONE) {
                nextEmptySlot = 0;
                debugManager.log("checkEmptySlot %d", nextEmptySlot);
                return true;
            }
        } else {
            if (artifactColorArray[2] == ArtifactColor.NONE) {
                nextEmptySlot = 2;
                debugManager.log("checkEmptySlot %d", nextEmptySlot);
                return true;
            } else if (artifactColorArray[1] == ArtifactColor.NONE) {
                nextEmptySlot = 1;
                debugManager.log("checkEmptySlot %d", nextEmptySlot);
                return true;
            } else if (artifactColorArray[0] == ArtifactColor.NONE) {
                nextEmptySlot = 0;
                debugManager.log("checkEmptySlot %d", nextEmptySlot);
                return true;
            }
        }

        debugManager.spindexer("no empty sloT");
        debugManager.log("NO EmptySlot %d", nextEmptySlot);
        return false;
    }

    public boolean turnEmptySlotToIntake() {
        debugManager.spindexer("turnEmptySlotToIntake", "%d", nextEmptySlot);
        debugManager.log("turnEmptySlotToIntake %s", nextEmptySlot);
        double position = getIndexerPosition();

        if (nextEmptySlot == 0) {
            if (position != POSITION_INDEXER_SERVO_SLOT_ZERO_INTAKE) {
                rotateToPosition(POSITION_INDEXER_SERVO_SLOT_ZERO_INTAKE);
                return true;
            }
        } else if (nextEmptySlot == 1) {
            if (position != POSITION_INDEXER_SERVO_SLOT_ONE_INTAKE) {
                rotateToPosition(POSITION_INDEXER_SERVO_SLOT_ONE_INTAKE);
                return true;
            }
        } else if (nextEmptySlot == 2) {
            if (position != POSITION_INDEXER_SERVO_SLOT_TWO_INTAKE) {
                rotateToPosition(POSITION_INDEXER_SERVO_SLOT_TWO_INTAKE);
                return true;
            }
        }
        return false;
    }

    public void positionForOuttake() {
        double position = getIndexerPosition();

        if (position <= POSITION_INDEXER_SERVO_SLOT_ONE_INTAKE
                && position != POSITION_INDEXER_SERVO_SLOT_ZERO_OUTPUT) {
            rotateToPosition(POSITION_INDEXER_SERVO_SLOT_ZERO_OUTPUT);
        }
        else {
            rotateToPosition(POSITION_INDEXER_SERVO_SLOT_ONE_OUTPUT);
        }
    }

    public void positionForIntake(){
        double position = getIndexerPosition();

        if (position <= POSITION_INDEXER_SERVO_SLOT_ONE_INTAKE
            && position != POSITION_INDEXER_SERVO_SLOT_TWO_INTAKE) {
            rotateToPosition(POSITION_INDEXER_SERVO_SLOT_TWO_INTAKE);
        }
        else {
            rotateToPosition(POSITION_INDEXER_SERVO_SLOT_ZERO_INTAKE);
        }
    }

    public void turnToClosestIntake(){
        double position = getAxonServoPosition();
        debugManager.spindexer("turnToClosestIntake", "%.2f", position);
        debugManager.log("turnToClosestIntake %.2f", position);

        if (position <= POSITION_INDEXER_SERVO_SLOT_ZERO_OUTPUT
            && position != POSITION_INDEXER_SERVO_SLOT_TWO_INTAKE) {
            rotateToPosition(POSITION_INDEXER_SERVO_SLOT_TWO_INTAKE);
        }
        else if (position <= POSITION_INDEXER_SERVO_SLOT_TWO_OUTPUT
            && position != POSITION_INDEXER_SERVO_SLOT_ONE_INTAKE) {
            rotateToPosition(POSITION_INDEXER_SERVO_SLOT_ONE_INTAKE);
        }
        else if (position != POSITION_INDEXER_SERVO_SLOT_ZERO_INTAKE){
            rotateToPosition(POSITION_INDEXER_SERVO_SLOT_ZERO_INTAKE);
        }
    }

    public boolean atIntake() {
        double position = getIndexerPosition();
        debugManager.log("!atIntake() %.2f", position);

        if (position == POSITION_INDEXER_SERVO_SLOT_ONE_INTAKE
                || position == POSITION_INDEXER_SERVO_SLOT_ZERO_INTAKE
                || position == POSITION_INDEXER_SERVO_SLOT_TWO_INTAKE) {
            return true;
        }
        else {
            return false;
        }
    }

    public Boolean moveToOuttake() {
        debugManager.spindexer("moveToOuttake");
        debugManager.log("moveToOuttake %d", nextShootSlot);
        if (nextShootSlot == 0) {
            if (getIndexerPosition() != POSITION_INDEXER_SERVO_SLOT_ZERO_OUTPUT) {
                rotateToPosition(POSITION_INDEXER_SERVO_SLOT_ZERO_OUTPUT);
                return true;
            }
        } else if (nextShootSlot == 1) {
            if (getIndexerPosition() != POSITION_INDEXER_SERVO_SLOT_ONE_OUTPUT) {
                rotateToPosition(POSITION_INDEXER_SERVO_SLOT_ONE_OUTPUT);
                return true;
            }
        } else if (nextShootSlot == 2) {
            if (getIndexerPosition() != POSITION_INDEXER_SERVO_SLOT_TWO_OUTPUT) {
                rotateToPosition(POSITION_INDEXER_SERVO_SLOT_TWO_OUTPUT);
                return true;
            }
        }
        return false;
    }

    public void updateAfterShoot() {
        // the ball has been shot in nextShootSlot
        debugManager.spindexer("updateAfterShoot: next shoot slot", "%d", nextShootSlot);
        debugManager.log("updateAfterShoot: next shoot slot %d", nextShootSlot);
        artifactColorArray[nextShootSlot] = ArtifactColor.NONE;
    }

    // find the next ball to shoot
    public boolean findABall() {

        if (artifactColorArray[0] == ArtifactColor.NONE
            && artifactColorArray[1] == ArtifactColor.NONE
            && artifactColorArray[2] == ArtifactColor.NONE) {
            return false;
        }
        double position = getIndexerPosition();
        if (position <= POSITION_INDEXER_SERVO_SLOT_ZERO_OUTPUT) {
            //is there a ball at ZERO?
            if (artifactColorArray[0] != ArtifactColor.NONE) {
                nextShootSlot = 0;
                return true;
            } else if (artifactColorArray[2] != ArtifactColor.NONE) {
                nextShootSlot = 2;
                return true;
            } else if (artifactColorArray[1] != ArtifactColor.NONE) {
                nextShootSlot = 1;
                return true;
            }
        } else if (position <= POSITION_INDEXER_SERVO_SLOT_TWO_OUTPUT) {
            //is there a ball at one?
            if (artifactColorArray[2] != ArtifactColor.NONE) {
                nextShootSlot = 2;
                return true;
            } else if (artifactColorArray[1] != ArtifactColor.NONE) {
                nextShootSlot = 1;
                return true;
            } else if (artifactColorArray[0] != ArtifactColor.NONE) {
                nextShootSlot = 0;
                return true;
            }
        } else if (position <= POSITION_INDEXER_SERVO_SLOT_ONE_OUTPUT) {
            //is there a ball at two?
            if (artifactColorArray[1] != ArtifactColor.NONE) {
                nextShootSlot = 1;
                return true;
            } else if (artifactColorArray[2] != ArtifactColor.NONE) {
                nextShootSlot = 2;
                return true;
            } else if (artifactColorArray[0] != ArtifactColor.NONE) {
                nextShootSlot = 0;
                return true;
            }
        }
        return false;
    }

    public void buildShootQueueNoColor(LinkedList<Integer> shootQueue){
        // build the shoot queue according
        double position = getIndexerPosition();
        if (position <= POSITION_INDEXER_SERVO_SLOT_ZERO_OUTPUT) {
            //is there a ball at ZERO?
            if (artifactColorArray[0] != ArtifactColor.NONE) {
                shootQueue.add(0);
            }
            if (artifactColorArray[2] != ArtifactColor.NONE) {
                shootQueue.add(2);
            }
            if (artifactColorArray[1] != ArtifactColor.NONE) {
                shootQueue.add(1);
            }
        } else if (position <= POSITION_INDEXER_SERVO_SLOT_TWO_OUTPUT) {
            //is there a ball at one?
            if (artifactColorArray[2] != ArtifactColor.NONE) {
                shootQueue.add(2);
            }
            if (artifactColorArray[1] != ArtifactColor.NONE) {
                shootQueue.add(1);
            }
            if (artifactColorArray[0] != ArtifactColor.NONE) {
                shootQueue.add(0);
            }
        } else if (position <= POSITION_INDEXER_SERVO_SLOT_ONE_OUTPUT) {
            //is there a ball at two?
            if (artifactColorArray[1] != ArtifactColor.NONE) {
                shootQueue.add(1);
            }
            if (artifactColorArray[2] != ArtifactColor.NONE) {
                shootQueue.add(2);
            }
            if (artifactColorArray[0] != ArtifactColor.NONE) {
                shootQueue.add(0);
            }
        }
    }

    // Check to see if there is any ball by distance sensing
    public boolean isBallAtIntake() {

        double dis1 = ((DistanceSensor) colorSensorIntakeL).getDistance(DistanceUnit.CM);
        double dis2 = ((DistanceSensor) colorSensorIntakeR).getDistance(DistanceUnit.CM);
        NormalizedRGBA colorSensorNormalizedColors1 = colorSensorIntakeL.getNormalizedColors();;
        NormalizedRGBA colorSensorNormalizedColors2 = colorSensorIntakeR.getNormalizedColors();;
        double position = getIndexerPosition();
        debugManager.spindexer("isBallAtIntake colorSensorIntakeL", "%.2f", dis1);
        debugManager.spindexer("isBallAtIntake colorSensorIntakeR", "%.2f", dis2);
        RobotLog.d("intake ball at %.2f, %.2f %.2f %.2f", dis1, dis2,colorSensorNormalizedColors1.alpha, colorSensorNormalizedColors2.alpha);

        if ((dis1 < 2.5 || dis2 < 2.5) && (colorSensorNormalizedColors1.alpha > 0.75 || colorSensorNormalizedColors2.alpha > 0.75)) {
            if (colorSensorNormalizedColors1.blue > colorSensorNormalizedColors1.green) {
                if (position == POSITION_INDEXER_SERVO_SLOT_ZERO_INTAKE)
                    artifactColorArray[0] = ArtifactColor.PURPLE;
                else if (position == POSITION_INDEXER_SERVO_SLOT_ONE_INTAKE)
                    artifactColorArray[1] = ArtifactColor.PURPLE;
                else if (position == POSITION_INDEXER_SERVO_SLOT_TWO_INTAKE)
                    artifactColorArray[2] = ArtifactColor.PURPLE;
            }
            else {
                if (position == POSITION_INDEXER_SERVO_SLOT_ZERO_INTAKE)
                    artifactColorArray[0] = ArtifactColor.GREEN;
                else if (position == POSITION_INDEXER_SERVO_SLOT_ONE_INTAKE)
                    artifactColorArray[1] = ArtifactColor.GREEN;
                else if (position == POSITION_INDEXER_SERVO_SLOT_TWO_INTAKE)
                    artifactColorArray[2] = ArtifactColor.GREEN;
            }
            //todo RobotLog.d("intake ball at %.2f color %s %s %s", position, artifactColorArray[0],artifactColorArray[1], artifactColorArray[2]);
            return true;
        } else {
            return false;
        }
    }

    // Check to see if there is any ball by distance sensing
    public boolean isBallAtIntakeFast() {
        double dis1 = ((DistanceSensor) colorSensorIntakeL).getDistance(DistanceUnit.CM);
        double dis2 = ((DistanceSensor) colorSensorIntakeR).getDistance(DistanceUnit.CM);
        double position = getIndexerPosition();
        debugManager.spindexer("isBallAtIntakeFast colorSensorIntakeL", "%.2f", dis1);
        debugManager.spindexer("isBallAtIntakeFast colorSensorIntakeR", "%.2f", dis2);
        debugManager.log("intake ball at %.2f, %.2f", dis1, dis2);

        if (dis1 < 3.5 || dis2 < 3.5) {
                if (position == POSITION_INDEXER_SERVO_SLOT_ZERO_INTAKE)
                    artifactColorArray[0] = ArtifactColor.UNKNOWN;
                else if (position == POSITION_INDEXER_SERVO_SLOT_ONE_INTAKE)
                    artifactColorArray[1] = ArtifactColor.UNKNOWN;
                else if (position == POSITION_INDEXER_SERVO_SLOT_TWO_INTAKE)
                    artifactColorArray[2] = ArtifactColor.UNKNOWN;
//           debugManager.log("intake ball at %.2f color %s %s %s",
//               position,
//               artifactColorArray[0],
//               artifactColorArray[1],
//               artifactColorArray[2]);
            return true;
        } else {
            return false;
        }
    }


//
//    private ArtifactColor getPredictedColorTeleOp(NormalizedRGBA sensor1RGBA, NormalizedRGBA sensor2RGBA, double sensor1Distance, double sensor2Distance) {
//
//        ArtifactColor sensor1DetectedColor;
//        //telemetry.addData("sensor1Distance", sensor1Distance);
//        //telemetry.addData("sensor2Distance", sensor2Distance);
//
//        if (sensor1Distance > 6.5) {
//            sensor1DetectedColor = ArtifactColor.NONE;
//        } else if (sensor1RGBA.blue > sensor1RGBA.green) {
//            sensor1DetectedColor = ArtifactColor.PURPLE;
//        } else {
//            sensor1DetectedColor = ArtifactColor.GREEN;
//        }
//
//        ArtifactColor sensor2DetectedColor;
//
//        if (sensor2Distance > 6.5) {
//            sensor2DetectedColor = ArtifactColor.NONE;
//        } else if (sensor2RGBA.blue > sensor2RGBA.green) {
//            sensor2DetectedColor = ArtifactColor.PURPLE;
//        } else {
//            sensor2DetectedColor = ArtifactColor.GREEN;
//        }
//
//        if (sensor1DetectedColor == sensor2DetectedColor) {
//            return sensor1DetectedColor;
//        } else if (sensor2DetectedColor == ArtifactColor.NONE) {
//            return sensor1DetectedColor;
//        } else if (sensor1DetectedColor == ArtifactColor.NONE) {
//            return sensor2DetectedColor;
//        } else {
//            return ArtifactColor.UNKNOWN;
//        }
//    }
//
//    public void updateUnknowBall() {
//          double position = getIndexerPosition();
//          if (artifactColorArray[0] == ArtifactColor.UNKNOWN) {
//              if (position == POSITION_INDEXER_SERVO_SLOT_ZERO_INTAKE) {
//                  updateBallColorAtIntake(position);
//              } else if (position == POSITION_INDEXER_SERVO_SLOT_ONE_INTAKE) {
//                  updateBallColorAtBackL(position);
//              } else if (position == POSITION_INDEXER_SERVO_SLOT_TWO_INTAKE) {
//                  updateBallColorAtBackR(position);
//              }
//          } else if (artifactColorArray[1] == ArtifactColor.UNKNOWN) {
//              if (position == POSITION_INDEXER_SERVO_SLOT_ZERO_INTAKE) {
//                  updateBallColorAtBackR(position);
//              } else if (position == POSITION_INDEXER_SERVO_SLOT_ONE_INTAKE) {
//                  updateBallColorAtIntake(position);
//              } else if (position == POSITION_INDEXER_SERVO_SLOT_TWO_INTAKE) {
//                  updateBallColorAtBackL(position);
//              }
//          } else if (artifactColorArray[2] == ArtifactColor.UNKNOWN) {
//              if (position == POSITION_INDEXER_SERVO_SLOT_ZERO_INTAKE) {
//                  updateBallColorAtBackL(position);
//              } else if (position == POSITION_INDEXER_SERVO_SLOT_ONE_INTAKE) {
//                  updateBallColorAtBackR(position);
//              } else if (position == POSITION_INDEXER_SERVO_SLOT_TWO_INTAKE) {
//                  updateBallColorAtIntake(position);
//              }
//          }
//
//    }

    public void setNextShootSlot(int slot) {
        nextShootSlot = slot;
    }

    public boolean axonAtIntake() {
        double position = getAxonServoPosition();
        debugManager.log("axonAtIntake() %.2f", position);

        if (Math.abs(position - POSITION_INDEXER_SERVO_SLOT_ONE_INTAKE) <= 0.03
            || Math.abs(position - POSITION_INDEXER_SERVO_SLOT_ZERO_INTAKE) <= 0.03
            || Math.abs(position - POSITION_INDEXER_SERVO_SLOT_TWO_INTAKE) <= 0.03) {
            return true;
        }
        else {
            return false;
        }
    }

    public boolean confirmColorMatch() {
        ArtifactColor[] preArtifactColorArray = artifactColorArray.clone();
        updateColorAllSlots();
        if(Arrays.equals(preArtifactColorArray, artifactColorArray)) {
            return true;
        }
        return false;
    }

    public int countArtifacts() {
        int count = 0;
        for (int i = 0; i <= 2; i++) {
            if (artifactColorArray[i] != ArtifactColor.NONE) count++;
        }
        return count;
    }

}
