package org.firstinspires.ftc.teamcode.autonomous.archived;

import static com.pedropathing.paths.HeadingInterpolator.lazy;
import static com.pedropathing.paths.HeadingInterpolator.linear;

import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.InvertedFTCCoordinates;
import com.pedropathing.ftc.PoseConverter;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.common.Robot;
import org.firstinspires.ftc.teamcode.common.RobotStaticValuesClass;
import org.firstinspires.ftc.teamcode.common.subsystems.Indexer;
import org.firstinspires.ftc.teamcode.common.util.ArtifactColor;
import org.firstinspires.ftc.teamcode.pedropathing.Constants;
import org.firstinspires.ftc.teamcode.pedropathing.commands.Paths;
import org.firstinspires.ftc.teamcode.pedropathing.commands.SubsystemCommands;

import java.util.HashMap;
import java.util.LinkedList;

@Disabled
@Autonomous(name = "Blue Front Pedro Pathing", group = "0Comp")
public class BlueFrontPedroPathingAuto extends OpMode {

    Follower follower;
    Robot robot;

    private enum State {
        INIT,
        READY,
        GO_TO_SHOOT_POS,
        SHOOT_PRELOAD,
        MID_PICKUP_GATE,
        SHOOT,
        CLOSE_PICKUP,
        FAR_PICKUP,
        WAIT_SHOOT_POS,
        LEAVE,
        FINISHED
    }

    private LinkedList<State> order = new LinkedList<>();
    private LinkedList<State> endLoop = new LinkedList<>();


    ArtifactColor[] motif = new ArtifactColor[] {ArtifactColor.GREEN, ArtifactColor.PURPLE, ArtifactColor.PURPLE};
    ArtifactColor[] motif_new = null;
    boolean motifFound = false;
    boolean staticDataSaved = false;




    private State state;
    int currentState = 0;

    private State getNextState() {
        if (currentState < order.size()) {
            return order.get(currentState++);
        }
        else if (endLoop.size() > 0) {
            return endLoop.get((currentState++ - order.size()) % endLoop.size());
        }
        else  {
            return State.FINISHED;
        }
    }

    Paths paths;

    @Override
    public void init() {
        state = State.INIT;

        follower = Constants.createFollower(hardwareMap);
        robot = new Robot(hardwareMap, telemetry, false);

        // Limelight
        robot.getLauncher().setLimelightPipeline(Robot.LLPipelines.OBELISK.ordinal());

        follower.setStartingPose(new Pose(17.766, 120.935, Math.toRadians(53.5)));

        SubsystemCommands subsystemCommands = new SubsystemCommands(robot);
        paths = new Paths(follower);


        rows.put(0, true);
        rows.put(1, true);
        rows.put(2, true);

        labels.put(0, "Middle Mark + Gate"); // TODO add Gate
        labels.put(1, "Close Mark");
        labels.put(2, "Far Mark");
        follower.update();
    }

    private int maxRows = 3;
    private int currentRow = 0;
    private HashMap<Integer, Boolean> rows = new HashMap<>(4);
    private HashMap<Integer, String> labels = new HashMap<>(4);

    Gamepad currentGamepad1 = new Gamepad();
    Gamepad previousGamepad1 = new Gamepad();
    Gamepad currentGamepad2 = new Gamepad();
    Gamepad previousGamepad2 = new Gamepad();


    @Override
    public void init_loop() {

        previousGamepad1.copy(currentGamepad1);
        previousGamepad2.copy(currentGamepad2);
        currentGamepad1.copy(gamepad1);
        currentGamepad2.copy(gamepad2);

        robot.getLauncher().autoUpdateTurretPID(-43);

        if (state == State.INIT) {
            for (int i = 0; i < maxRows; i++) {
                if (i == currentRow) {
                    telemetry.addData("> " + labels.get(i), rows.get(i));
                }
                else {
                    telemetry.addData("   " + labels.get(i), rows.get(i));
                }
            }

            if ((currentGamepad1.dpad_up && !previousGamepad1.dpad_up)
                || (currentGamepad2.dpad_up && !previousGamepad2.dpad_up)) {
                currentRow--;
            }

            if ((currentGamepad1.dpad_down && !previousGamepad1.dpad_down)
                || (currentGamepad2.dpad_down && !previousGamepad2.dpad_down)) {
                currentRow++;
            }
            if (currentRow >= maxRows) {
                currentRow = maxRows - 1;
            }
            else if (currentRow < 0) {
                currentRow = 0;
            }

            if ((currentGamepad1.dpad_left && !previousGamepad1.dpad_left)
                || (currentGamepad2.dpad_left && !previousGamepad2.dpad_left)
                || (currentGamepad1.dpad_right && !previousGamepad1.dpad_right)
                || (currentGamepad2.dpad_right && !previousGamepad2.dpad_right)) {
                rows.put(currentRow, !rows.get(currentRow));
            }

            if (currentGamepad1.a || currentGamepad2.a) {
                order.add(State.GO_TO_SHOOT_POS);
                // order.add(State.WAIT_SHOOT_POS);
                order.add(State.SHOOT_PRELOAD);
                if (rows.get(0)) {
                    order.add(State.MID_PICKUP_GATE);
                    order.add(State.SHOOT);
                }
                if (rows.get(1)) {
                    order.add(State.CLOSE_PICKUP);
                    order.add(State.SHOOT);
                }
                if (rows.get(2)) {
                    order.add(State.FAR_PICKUP);
                    order.add(State.SHOOT);
                }
                order.add(State.LEAVE);
                state = State.READY;
            }
        }
    }

    @Override
    public void start() {
        if (state != State.READY) {
            // throw new RuntimeException("Program was not locked in before running");
            RobotLog.d("Program was not locked in before Running");
            telemetry.addData("Program was not locked in before Running", state);
            order.add(State.GO_TO_SHOOT_POS);
            order.add(State.SHOOT_PRELOAD);
            if (rows.get(0)) {
                order.add(State.MID_PICKUP_GATE);
                order.add(State.SHOOT);
            }
            if (rows.get(1)) {
                order.add(State.CLOSE_PICKUP);
                order.add(State.SHOOT);
            }
            if (rows.get(2)) {
                order.add(State.FAR_PICKUP);
                order.add(State.SHOOT);
            }
            order.add(State.LEAVE);
            state = State.READY;
        }
        state = getNextState();
    }

    boolean newState_Shoot = true;
    boolean newState_Drive = true;

    @Override
    public void loop() {
//        telemetry.addData("Status", "Running");
//
//        telemetry.addData("State", state);
//        telemetry.addData("Shoot State", shootState);
//        telemetry.addData("Drive State", driveState);

        switch (state) {
            case SHOOT_PRELOAD:
                // RobotLog.d ("S: SHOOT_PRELOAD");
                updateShoot(follower.getPose(), -43);
                break;
            case GO_TO_SHOOT_POS:
                follower.followPath(paths.getBlueCloseStartToShoot2(), false);
                state = state.WAIT_SHOOT_POS; // Wait for position
                break;
            case WAIT_SHOOT_POS:
                // Let's read Limelight in here
                if (!motifFound) {
                    motif_new = robot.getLauncher().getMotifPattern(false);
                    if (motif_new != null) {
                        motif = motif_new;
                        RobotLog.d("Motif Pattern Found %s:%s:%s", motif[0].toString(), motif[1].toString(), motif[2].toString());
                        motifFound = true;
                    }
                }
                if (!follower.isBusy()){
                    state = getNextState();
                }
                break;
            case MID_PICKUP_GATE:
                RobotLog.d ("S: MID_PICKUP_GATE");
                updateDrive(paths.getBlueClosePickupSecondMark(), paths.getBlueCloseReturnFromSecondMark(), 150);
                break;

            case SHOOT:
                RobotLog.d ("S: SHOOT");
                updateShoot(follower.getPose(), -43);
                break;
            case FAR_PICKUP:
                RobotLog.d ("S: FAR_PICKUP");
                updateDrive(paths.getBlueClosePickupThirdMark(), paths.getBlueCloseReturnFromThirdMark(), 150);
                break;
            case CLOSE_PICKUP:
                RobotLog.d ("S: CLOSE_PICKUP");
                updateDrive(paths.getBlueClosePickupFirstMark(), paths.getBlueCloseReturnFromFirstMark(), 150);
                break;
            case LEAVE:
                follower.followPath(follower.pathBuilder()
                    .addPath(new BezierLine(() -> follower.getPose(), new Pose(54, 126)))
                    .setHeadingInterpolation(lazy(() -> linear(follower.getHeading(), Math.toRadians(180),0.8)))
                    .build(), false);
                break;
        }

        follower.update();

        // convert the and update the current pose to global
        // RobotStaticValuesClass.savedPose = InvertedFTCCoordinates.INSTANCE.convertFromPedro(new Pose(follower.getPose().getX(), follower.getPose().getY(), follower.getPose().getHeading()));


//        RobotLog.d("L:fx:" + RobotStaticValuesClass.savedPose.getX(DistanceUnit.INCH) + ", y:" + RobotStaticValuesClass.savedPose.getY(DistanceUnit.INCH) + ", h:" + RobotStaticValuesClass.savedPose.getHeading(AngleUnit.RADIANS));
//        RobotLog.d("L:px:"+ follower.getPose().getX() + ", y:" + follower.getPose().getY() + ", h:" + follower.getPose().getHeading());
//        RobotLog.d("L:ta:"+ robot.getLauncher().getCurrentAngleOffset());

        // if (!staticDataSaved){
            if (motif[0]==ArtifactColor.PURPLE && motif[1]==ArtifactColor.GREEN && motif[2]==ArtifactColor.PURPLE){
                RobotStaticValuesClass.savedObelisk =    RobotStaticValuesClass.Obelisk.PGP;
            }
            else if (motif[0]==ArtifactColor.PURPLE && motif[1]==ArtifactColor.PURPLE && motif[2]==ArtifactColor.GREEN){
                RobotStaticValuesClass.savedObelisk =    RobotStaticValuesClass.Obelisk.PPG;
            }
            else
            {
                RobotStaticValuesClass.savedObelisk = RobotStaticValuesClass.Obelisk.GPP;
            }
            RobotStaticValuesClass.autoCompleted = true;
            staticDataSaved = true;
        // }



        // RobotLog.d("States: " + state + ", " + shootState + ", " + driveState + ", " + newState_Drive);

        telemetry.update();
    }

    @Override
    public void stop() {
        RobotStaticValuesClass.savedPose = PoseConverter.poseToPose2D(follower.getPose(), InvertedFTCCoordinates.INSTANCE);
        RobotStaticValuesClass.turretAngleOffset = robot.getLauncher().getCurrentAngleOffset();
    }

    private enum ShootState {
        INIT,
        PREPARING,
        SHOOTING_0,
        FINISHED
    }

    ShootState shootState = ShootState.INIT;

    private void updateShoot(Pose holdPose, double turretAngle) {
        if (newState_Shoot) {
            shootState = ShootState.INIT;
        }

        switch (shootState) {
            case INIT:
                // RobotLog.d ("SS: INIT");
                robot.getLauncher().setAutoVelocity(1280);

                // robot.getLauncher().autoUpdateTurretPID(turretAngle);
                robot.updateTurretAngleAuto();
                follower.holdPoint(holdPose);

                robot.setRobotState(Robot.RobotInOutState.OUTTAKE);

                robot.getIndexer().autoFillColorArray();

                if (robot.getLauncher().getLauncherVelocity() >= (robot.getLauncher().getLauncherTargetVelocity()-20) /*&& Math.abs(robot.getLauncher().getTurretDegrees() - turretAngle) < 2.5*/) {
                    shootState = ShootState.SHOOTING_0;
                }
                else {
                    shootState = ShootState.PREPARING;
                }
                break;
            case PREPARING:

                // RobotLog.d ("SS: PREPARING LV:%.2f TA:%.2f", robot.getLauncher().getLauncherVelocity(), robot.getLauncher().getTurretDegrees());
                // telemetry.addData("Launcher Velocity", robot.getLauncher().getLauncherVelocity());
                // telemetry.addData("Turret Angle", robot.getLauncher().getTurretDegrees());


                // robot.getLauncher().autoUpdateTurretPID(turretAngle);
                robot.updateTurretAngleAuto();

                if (robot.getLauncher().getLauncherVelocity() >= (robot.getLauncher().getLauncherTargetVelocity()-20) /*&& Math.abs(robot.getLauncher().getTurretDegrees() - turretAngle) < 2.5*/) {
                    shootState = ShootState.SHOOTING_0;
                }
                break;
            case SHOOTING_0:
                // RobotLog.d ("SS: SHOOTING_0");
                // robot.getLauncher().autoUpdateTurretPID(turretAngle);
                robot.updateTurretAngleAuto();

                robot.shootAllBallsAuto();

                if (robot.isNoArtifacts()) {
                    shootState = ShootState.FINISHED;
                }
                break;
            case FINISHED:
                // RobotLog.d ("SS: FINISHED");
                robot.getLauncher().setAutoVelocity(0);
                // telemetry.addData("Shoot State", shootState);
                state = getNextState();
                newState_Shoot = true;
        }

        if (shootState != ShootState.FINISHED) {
            newState_Shoot = false;
        }
    }

    private enum DriveState {
        INIT,
        PREPARE,
        PICKUP_1,
        PICKUP_2,
        PICKUP_0,
        PICKUP_2_RETURN,
        PREP_FOR_SHOOT_INIT,
        PREP_FOR_SHOOT,
        FINISHED
    }

    DriveState driveState = DriveState.INIT;

    ElapsedTime delayTimer = new ElapsedTime();
    ElapsedTime intakeHoldTimer = new ElapsedTime();

    private boolean resetTimer;


    private void updateDrive(PathChain pickup, PathChain returnToPos, long midDelay) {
        if (newState_Drive) {
            driveState = DriveState.INIT;
        }

        switch (driveState) {
            case INIT:
                // RobotLog.d ("SD: INIT");
                follower.followPath(pickup, 0.8, false);
                resetTimer = false;
                robot.getIntake().setIntakeMotorPower(1);
                robot.getIndexer().rotateToPosition(Indexer.POSITION_INDEXER_SERVO_SLOT_ZERO_INTAKE);
                driveState = DriveState.PREPARE;
                break;
            case PREPARE:
                // RobotLog.d ("SD: PREPARE");
                if (robot.getIndexer().getIndexerServoAtPosition(Indexer.POSITION_INDEXER_SERVO_SLOT_ZERO_INTAKE, 0.05)) {
                    driveState = DriveState.PICKUP_0;
                }

                if (!follower.isBusy()) {
                    if (midDelay == 0) {
                        driveState = DriveState.PREP_FOR_SHOOT_INIT;
                    }
                    else if (!resetTimer) {
                        delayTimer.reset();
                        resetTimer = true;
                    }
                    else if (delayTimer.milliseconds() >= midDelay){
                        driveState = DriveState.PREP_FOR_SHOOT_INIT;
                    }
                }

                break;
            case PICKUP_0:
                // RobotLog.d ("SD: PICKUP_0");
                if (robot.getIndexer().getIndexerServoAtPosition(Indexer.POSITION_INDEXER_SERVO_SLOT_ZERO_INTAKE, 0.05)
                    && robot.getIndexer().isBallAtIntakeFast()) {

                    driveState = DriveState.PICKUP_1;
                    robot.getIndexer().rotateToPosition(Indexer.POSITION_INDEXER_SERVO_SLOT_ONE_INTAKE);
                }

                if (!follower.isBusy()) {
                    if (midDelay == 0) {
                        driveState = DriveState.PREP_FOR_SHOOT_INIT;
                    }
                    else if (!resetTimer) {
                        delayTimer.reset();
                        resetTimer = true;
                    }
                    else if (delayTimer.milliseconds() >= midDelay){
                        driveState = DriveState.PREP_FOR_SHOOT_INIT;
                    }
                }

                break;
            case PICKUP_1:
                // RobotLog.d ("SD: PICKUP_1");
                if (robot.getIndexer().getIndexerServoAtPosition(Indexer.POSITION_INDEXER_SERVO_SLOT_ONE_INTAKE, 0.05) &&
                    robot.getIndexer().isBallAtIntakeFast()) {

                    driveState = DriveState.PICKUP_2;
                    robot.getIndexer().rotateToPosition(Indexer.POSITION_INDEXER_SERVO_SLOT_TWO_INTAKE);
                }
                if (!follower.isBusy()) {
                    if (midDelay == 0) {
                        driveState = DriveState.PREP_FOR_SHOOT_INIT;
                    }
                    else if (!resetTimer) {
                        delayTimer.reset();
                        resetTimer = true;
                    }
                    else if (delayTimer.milliseconds() >= midDelay){
                        driveState = DriveState.PREP_FOR_SHOOT_INIT;
                    }
                }
                break;
            case PICKUP_2:
                // RobotLog.d ("SD: PICKUP_2");
                if (robot.getIndexer().getIndexerServoAtPosition(Indexer.POSITION_INDEXER_SERVO_SLOT_TWO_INTAKE, 0.05)
                    && robot.getIndexer().isBallAtIntakeFast()) {

                    driveState = DriveState.PREP_FOR_SHOOT_INIT;
                    robot.getIndexer().rotateToPosition(Indexer.POSITION_INDEXER_SERVO_SLOT_ZERO_OUTPUT);
                    robot.getLauncher().setAutoVelocity(1280);

                }
                if (!follower.isBusy()) {
                    if (midDelay == 0) {
                        follower.followPath(returnToPos);
                        driveState = DriveState.PICKUP_2_RETURN;
                        intakeHoldTimer.reset(); // start the holdover timer for intake
                    }
                    else if (!resetTimer) {
                        delayTimer.reset();
                        resetTimer = true;
                    }
                    else if (delayTimer.milliseconds() >= midDelay){
                        follower.followPath(returnToPos);
                        driveState = DriveState.PICKUP_2_RETURN;
                        intakeHoldTimer.reset(); // start the holdover timer for intake
                    }
                }
                break;
            case PICKUP_2_RETURN:
                RobotLog.d ("SD: PICKUP_2_RETURN");
                if (robot.getIndexer().getIndexerServoAtPosition(Indexer.POSITION_INDEXER_SERVO_SLOT_TWO_INTAKE, 0.05)
                    && robot.getIndexer().isBallAtIntakeFast()) {

                    driveState = DriveState.PREP_FOR_SHOOT_INIT;
                    robot.getIndexer().rotateToPosition(Indexer.POSITION_INDEXER_SERVO_SLOT_ZERO_OUTPUT);
                    robot.getLauncher().setAutoVelocity(1280);

                }
                if (!follower.isBusy()) {
                    driveState = DriveState.FINISHED;
                }
                break;
            case PREP_FOR_SHOOT_INIT:
                // RobotLog.d ("SD: PREP_FOR_SHOOT_INIT");
                driveState = DriveState.PREP_FOR_SHOOT;
                robot.getLauncher().setAutoVelocity(1280);
                if (!follower.isBusy()) {
                    follower.followPath(returnToPos);
                }
                break;
            case PREP_FOR_SHOOT:
                // RobotLog.d ("SD: PREP_FOR_SHOOT");
                if (intakeHoldTimer.milliseconds() >= 3000) {
                    intakeHoldTimer.reset();
                    robot.getIntake().setIntakeMotorPower(0);
                }
                if (!follower.isBusy()) {

                    driveState = DriveState.FINISHED;
                }

                // robot.getLauncher().autoUpdateTurretPID(turretAngle);
                robot.updateTurretAngleAuto();
                break;
            case FINISHED:
                // RobotLog.d ("SD: FINISHED");
                state = getNextState();
                newState_Drive = true;
                break;
        }

        if (driveState != DriveState.FINISHED) {
            newState_Drive = false;
        }
    }
}
