package org.firstinspires.ftc.teamcode.autonomous.tasks;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.common.Robot;
import org.firstinspires.ftc.teamcode.common.subsystems.Indexer;
import org.firstinspires.ftc.teamcode.common.subsystems.Launcher;
import org.firstinspires.ftc.teamcode.tasks.BasicTask;
import org.firstinspires.ftc.teamcode.tasks.DeadlineTask;
import org.firstinspires.ftc.teamcode.tasks.FollowPathTask;
import org.firstinspires.ftc.teamcode.tasks.HoldPointTask;
import org.firstinspires.ftc.teamcode.tasks.InstantTask;
import org.firstinspires.ftc.teamcode.tasks.ParallelTask;
import org.firstinspires.ftc.teamcode.tasks.SequentialTask;
import org.firstinspires.ftc.teamcode.tasks.SleepTask;
import org.firstinspires.ftc.teamcode.tasks.Task;
import org.firstinspires.ftc.teamcode.tasks.WaitUntilTask;

public class AutoTaskMaker {
    Robot robot;
    Follower follower;
    public AutoTaskMaker(Robot robot, Follower follower) {
        this.robot = robot;
        this.follower = follower;
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
            new WaitUntilTask(() -> robot.getLauncher().getLauncherVelocity() >= velocity - 20)
        );
    }

    public Task setFarLauncherTask() {
        return spinUpLauncherTask(1610);
    }

    public Task setCloseLauncherTask() {
        return spinUpLauncherTask(1280);
    }

    public Task setTurretPositionTask(double angle) {
        return new BasicTask(() -> {}, () -> {
            robot.getLauncher().autoUpdateTurretPID(angle);
            return Math.abs(robot.getLauncher().getTurretDegrees() - angle) < 5;
        });
    }

    public Task setIndexerPositionTask(double position) {
        return new InstantTask(() -> robot.getIndexer().rotateToPosition(position));
    }

    public Task rotateIndexerTask(double position) {
        return new SequentialTask(
            setIndexerPositionTask(position),
            new WaitUntilTask(() -> robot.getIndexer().getIndexerServoAtPosition(position, 0.03))
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

    public Task setKickerUpTask() {
        return new SequentialTask(
            setKickerPositionTask(Launcher.POSITION_KICKER_SERVO_KICK_BALL),
            new SleepTask(Robot.WAIT_TIME_KICKER_UP)
        );
    }

    public Task setKickerDownTask() {
        return new SequentialTask(
            setKickerPositionTask(Launcher.POSITION_KICKER_SERVO_INIT),
            new SleepTask(Robot.WAIT_TIME_KICKER_DOWN)
        );
    }

    public Task waitUntilBallInIntakeTask() {
        return new WaitUntilTask(() -> robot.getIndexer().isBallAtIntake());
    }

    public enum Side {
        NEAR,
        FAR
    }

    public Task runPickupSequenceTask(PathChain pickupPath, PathChain returnPath, long delay, Side side) {
        return new DeadlineTask(
            new SequentialTask(
                new FollowPathTask(follower, pickupPath),
                new HoldPointTask(follower, pickupPath.endPose()),
                new SleepTask(delay),
                new FollowPathTask(follower, returnPath)
            ),
            new SequentialTask(
                startIntakeTask(),
                rotateIndexerIntakeTask(0),
                waitUntilBallInIntakeTask(),
                rotateIndexerIntakeTask(1),
                waitUntilBallInIntakeTask(),
                rotateIndexerIntakeTask(2),
                waitUntilBallInIntakeTask(),
                stopIntakeTask(),
                side.equals(Side.NEAR) ? setCloseLauncherTask() : setFarLauncherTask()
            )
        );
    }

    public Task runShootSequenceTask(Pose hold, Side side) {
        return new SequentialTask(
            new HoldPointTask(follower, hold),
            new ParallelTask(
                side.equals(Side.NEAR) ? setCloseLauncherTask() : setFarLauncherTask(),
                rotateIndexerOuttakeTask(0)
            ),
//            setKickerUpTask(),
//            setKickerDownTask(),
            rotateIndexerOuttakeTask(2),
//            setKickerUpTask(),
//            setKickerDownTask(),
            rotateIndexerOuttakeTask(1),
//            setKickerUpTask(),
//            setKickerDownTask(),
            setLauncherMotorVelocityTask(0)
        );
    }
}
