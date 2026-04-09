package org.firstinspires.ftc.teamcode.autonomous.tasks;

import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.InvertedFTCCoordinates;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.common.Robot;
import org.firstinspires.ftc.teamcode.common.RobotStaticValuesClass;
import org.firstinspires.ftc.teamcode.common.subsystems.Indexer;
import org.firstinspires.ftc.teamcode.common.subsystems.Launcher;
import org.firstinspires.ftc.teamcode.common.util.ArtifactColor;
import org.firstinspires.ftc.teamcode.tasks.BasicTask;
import org.firstinspires.ftc.teamcode.tasks.DeadlineTask;
import org.firstinspires.ftc.teamcode.tasks.DeferredTask;
import org.firstinspires.ftc.teamcode.tasks.FollowPathTask;
import org.firstinspires.ftc.teamcode.tasks.HoldPointTask;
import org.firstinspires.ftc.teamcode.tasks.InstantTask;
import org.firstinspires.ftc.teamcode.tasks.ParallelTask;
import org.firstinspires.ftc.teamcode.tasks.SequentialTask;
import org.firstinspires.ftc.teamcode.tasks.SleepTask;
import org.firstinspires.ftc.teamcode.tasks.Task;
import org.firstinspires.ftc.teamcode.tasks.WaitUntilTask;

import java.util.Arrays;

public class AutoTaskMaker {
    Robot robot;
    Follower follower;
    public AutoTaskMaker(Robot robot, Follower follower) {
        this.robot = robot;
        this.follower = follower;
    }

    public enum Team {
        RED,
        BLUE
    }

    public Task setIntakePowerTask(double power) {
        return new InstantTask(() -> robot.getIntake().setIntakeMotorPower(power));
    }

    public Task startIntakeTask() {
        return setIntakePowerTask(1);
    }

    public Task stopIntakeTask() {
        return setIntakePowerTask(0);
    }

    public Task reverseIntakeTask() {
        return setIntakePowerTask(-1);
    }

    public Task setLauncherMotorVelocityTask(double velocity) {
        return new InstantTask(() -> robot.getLauncher().setAutoVelocity(velocity));
    }

    public Task spinUpLauncherTask(double velocity) {
        return new SequentialTask(
            setLauncherMotorVelocityTask(velocity),
            new WaitUntilTask(() -> {
                robot.getTelemetry().addData("Velocity", velocity);
                return robot.getLauncher().getLauncherVelocity() >= velocity - 20;
            })
        );
    }

    public Task setFarLauncherTask() {
        return spinUpLauncherTask(1530);
    }

    public Task setCloseLauncherTask() {
        return spinUpLauncherTask(1310);
    }

    public Task setTurretPositionTask(double angle) {
        return new BasicTask(() -> {}, () -> {
            robot.getLauncher().autoUpdateTurretPID(angle);
            return Math.abs(robot.getLauncher().getTurretDegrees() - angle) < 2;
        });
    }

    public Task stopTurretTask() {
        return new InstantTask(() -> robot.getLauncher().setTurretPower0());
    }

    public Task setHoodPositionTask(double position) {
        return new InstantTask(() -> robot.getLauncher().setHoodServoPosition(position));
    }



    public Task setLauncherToGoalTask() {
        return new BasicTask(
            () -> {},
            () -> {
                robot.updateTurretAngleAuto();
                return false;
            }
        );
    }

    public Task setIndexerPositionTask(double position) {
        return new InstantTask(() -> robot.getIndexer().rotateToPosition(position));
    }

    public Task rotateIndexerTask(double position) {
        return new SequentialTask(
            setIndexerPositionTask(position),
            new WaitUntilTask(() -> {
                RobotLog.d("Indexer Position: position: %.4f, target: %.4f", robot.getIndexer().getIndexerPosition(), position);
                return robot.getIndexer().getIndexerServoAtPosition(position, 0.03);
            })
        );
    }

    public Task rotateIndexerIntakeTask(int slot) {
        if (slot == 0) {
            return rotateIndexerTask(Indexer.POSITION_INDEXER_SERVO_SLOT_ZERO_INTAKE);
        }
        else if (slot == 1) {
            return rotateIndexerTask(Indexer.POSITION_INDEXER_SERVO_SLOT_ONE_INTAKE);
        }
        else if (slot == 2) {
            return rotateIndexerTask(Indexer.POSITION_INDEXER_SERVO_SLOT_TWO_INTAKE);
        }
        else {
            throw new RuntimeException("Attempted to retrieve an unknown Indexer position");
        }
    }


    // As a note, best shooting order for speed is 1 -> 2 -> 0 or 0 -> 2 -> 1

    public Task rotateIndexerOuttakeTask(int slot) {
        if (slot == 0) {
            return rotateIndexerTask(Indexer.POSITION_INDEXER_SERVO_SLOT_ZERO_OUTPUT);
        }
        else if (slot == 1) {
            return rotateIndexerTask(Indexer.POSITION_INDEXER_SERVO_SLOT_ONE_OUTPUT);
        }
        else if (slot == 2) {
            return rotateIndexerTask(Indexer.POSITION_INDEXER_SERVO_SLOT_TWO_OUTPUT);
        }
        else {
            throw new RuntimeException("Attempted to retrieve an unknown Indexer position");
        }
    }

    public Task setKickerPositionTask(double position) {
        return new InstantTask(() -> robot.getLauncher().setKickerServoPosition(position));
    }

    @Deprecated
    public Task setKickerUpTask() {
        return new SequentialTask(
            setKickerPositionTask(Launcher.POSITION_KICKER_SERVO_KICK_BALL),
            new SleepTask(Robot.WAIT_TIME_KICKER_UP)
        );
    }

    @Deprecated
    public Task setKickerDownTask() {
        return new SequentialTask(
            setKickerPositionTask(Launcher.POSITION_KICKER_SERVO_INIT),
            new SleepTask(Robot.WAIT_TIME_KICKER_DOWN)
        );
    }

    public Task runElevatorTask() {
        return new BasicTask(
            () -> robot.getLauncher().elevateBall(),
            () -> !robot.getLauncher().elevatorRunning()
        );
    }

    public Task logTask(String value) {
        return new InstantTask(() -> RobotLog.d(value));
    }

    public Task waitUntilBallInIntakeTask() {
        return new WaitUntilTask(() -> robot.getIndexer().isBallAtIntakeFast());
    }

    public Task waitUntilTurretAimedTask() {
        return new WaitUntilTask(() -> robot.getLauncher().getAutoTurretAimed());
    }


    public Task detectObeliskTask()  {
        return new AprilTagTask(robot, robot.isRedSide());
    }

    public enum Side {
        NEAR,
        FAR
    }

    public Task runPickupSequenceTask(PathChain pickupPath, PathChain returnPath, long delay, Side side, Team team) {
        return new DeadlineTask(
            new ParallelTask(
            logTask("Task: RunPickupSequenceTask"),
            new SequentialTask(
                    new DeadlineTask(
                        new SequentialTask(
                            new FollowPathTask(follower, pickupPath, 0.8),
                            logTask("Pickup Task: Finished Path"),
                            new HoldPointTask(follower, pickupPath.endPose()),
                            logTask("Pickup Task: Hold Pose"),
                            new SleepTask(delay),
                            logTask("Pickup Task: Slept")
                        ),
                        new SequentialTask(
                            startIntakeTask(),
                            logTask("Pickup Task: Start Intake"),
                            rotateIndexerIntakeTask(0),
                            logTask("Pickup Task: Index to 0"),
                            waitUntilBallInIntakeTask(),
                            logTask("Pickup Task: Ball in Robot 0"),
                            rotateIndexerIntakeTask(1),
                            logTask("Pickup Task: Index to 1"),
                            waitUntilBallInIntakeTask(),
                            logTask("Pickup Task: Ball in Robot 1"),
                            rotateIndexerIntakeTask(2),
                            logTask("Pickup Task: Index to 2"),
                            waitUntilBallInIntakeTask(),
                            logTask("Pickup Task: Ball in Robot 2")
                        )
                    ),
                    logTask("Pickup Task: Finished First Deadline Task"),

                    new DeadlineTask(
                        new SequentialTask(
                            new FollowPathTask(follower, returnPath),
                            logTask("Pickup Task: Finish Return Path")
                        ),
                        new ParallelTask(
                            new SequentialTask(
                                reverseIntakeTask(),
                                logTask("Pickup Task: Reverse Intake"),

                                new SleepTask(1000),
                                logTask("Pickup Task: Slept"),

                                stopIntakeTask(),
                                logTask("Pickup Task: Intake Stopped")
                            ),
                            new SequentialTask(
                                side.equals(Side.NEAR) ? setCloseLauncherTask() : setFarLauncherTask(),
                                logTask("Pickup Task: Launcher Set")
                            )
                        )
                    ),
                    stopIntakeTask(),
                    logTask("Task: End of Pickup Sequence")
                )
            ),
            setLauncherToGoalTask()
        );
    }

    /**
     * Builds the shoot order at runtime after the obelisk has been detected.
     * The Supplier reads RobotStaticValuesClass.savedObelisk and the indexer colors
     * when init() is called — not at construction time.
     */
    public Task runShootSequenceForObeliskTask(Pose hold, Side side, Team team, int greenSlot) {
        return new DeferredTask(() -> {
            RobotStaticValuesClass.Obelisk obelisk = RobotStaticValuesClass.savedObelisk;

            int greenPosition;

            switch (obelisk) {
                case GPP:
                    greenPosition = 0;
                    break;
                case PGP:
                    greenPosition = 1;
                    break;
                case PPG:
                    greenPosition = 2;
                    break;
                default:
                    greenPosition = -1;
                    break;
            }

            int[] shootOrder = new int[]{0, 2, 1};

            // gp = 0 gs = 0: [0, 2, 1]
            // gp = 0 gs = 1: [1, 0, 2]
            // gp = 0 gs = 2: [2, 0, 1]

            //


            if (greenPosition >= 0 && greenSlot >= 0) {
                shootOrder[0] = greenPosition == 0 ? greenSlot : (greenSlot == 0 ? 2 : 0);
                shootOrder[2] = greenPosition == 2 ? greenSlot : (greenSlot == 1 ? 2 : 1);

                if (shootOrder[0] != 0 && shootOrder[2] != 0) {
                    shootOrder[1] = 0;
                }
                if (shootOrder[0] != 1 && shootOrder[2] != 1) {
                    shootOrder[1] = 1;
                }
                if (shootOrder[0] != 2 && shootOrder[2] != 2) {
                    shootOrder[1] = 2;
                }
            }

            int slot0 = shootOrder[0];
            int slot1 = shootOrder[1];
            int slot2 = shootOrder[2];

            RobotLog.d("Shoot Order: " + Arrays.toString(shootOrder));

            return new DeadlineTask(
                new SequentialTask(
                    logTask("Task: RunShootSequenceTask Obelisk: " + obelisk),
                    stopIntakeTask(),
                    new HoldPointTask(follower, hold),
                    new ParallelTask(
                        side == Side.FAR ? setFarLauncherTask() : setCloseLauncherTask(),
                        rotateIndexerOuttakeTask(slot0),
                        logTask("Shoot Task: Start Launcher and Rotate Indexer")
                    ),
                    logTask("Shoot Task: Index " + slot0),
                    runElevatorTask(),
                    logTask("Shoot Task: Shoot " + slot0),
                    rotateIndexerOuttakeTask(slot1),
                    logTask("Shoot Task: Index " + slot1),
                    runElevatorTask(),
                    logTask("Shoot Task: Shoot " + slot1),
                    rotateIndexerOuttakeTask(slot2),
                    logTask("Shoot Task: Index " + slot2),
                    runElevatorTask(),
                    logTask("Shoot Task: Shoot " + slot2),
                    setLauncherMotorVelocityTask(0),
                    logTask("Task: End of Shoot Sequence")
                ),
                setLauncherToGoalTask()
            );
        });
    }

    public Task runShootSequenceTask(Pose hold, Side side, Team team) {
        return new DeadlineTask(
            new SequentialTask(
                logTask("Task: RunShootSequenceTask"),
                stopIntakeTask(),
                new HoldPointTask(follower, hold),
                new ParallelTask(
                    side == Side.FAR ? setFarLauncherTask() : setCloseLauncherTask(),
                    rotateIndexerOuttakeTask(0)
                ),
                logTask("Shoot Task: Index 0"),
                runElevatorTask(),
                logTask("Shoot Task: Shoot 0"),
                rotateIndexerOuttakeTask(2),
                logTask("Shoot Task: Index 1"),
                runElevatorTask(),
                logTask("Shoot Task: Shoot 1"),
                rotateIndexerOuttakeTask(1),
                logTask("Shoot Task: Index 2"),
                runElevatorTask(),
                logTask("Shoot Task: Shoot 2"),
                setLauncherMotorVelocityTask(0)
            ),
            setLauncherToGoalTask(),
            logTask("Task: End of Shoot Sequence")
        );
    }
}
