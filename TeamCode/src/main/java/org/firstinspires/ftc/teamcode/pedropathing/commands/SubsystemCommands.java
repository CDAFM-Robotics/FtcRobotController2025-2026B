package org.firstinspires.ftc.teamcode.pedropathing.commands;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.Subsystem;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.common.Robot;
import org.firstinspires.ftc.teamcode.common.RobotStaticValuesClass;
import org.firstinspires.ftc.teamcode.common.subsystems.Indexer;
import org.firstinspires.ftc.teamcode.common.util.ArtifactColor;

import java.util.Collections;
import java.util.Set;


public class SubsystemCommands {
    Robot robot;

    public enum AutonomousTeam {
        RED,
        BLUE
    }

    public enum AutonomousSide {
        NEAR,
        FAR
    }


    public SubsystemCommands(Robot robot) {
        this.robot = robot;
    }

    public Command setPowerIntake(double power) {
        return new InstantCommand(() -> {
            robot.getIntake().setIntakeMotorPower(power);
            robot.getTelemetry().addData("Cool Intake Power", power);
            robot.getTelemetry().update();
        });
    }

    public Command startIntake() {
        return setPowerIntake(1);
    }

    public Command stopIntake() {
        return setPowerIntake(0);
    }

    public Command rotateIndexer(double position) {
        return new Command() {
            @Override
            public void initialize() {
                robot.getIndexer().rotateToPosition(position);
            }

            double indexerPosition;

            @Override
            public void execute() {
                indexerPosition = robot.getIndexer().getIndexerPosition();
            }

            @Override
            public boolean isFinished() {
                return Math.abs(position - indexerPosition) < 0.04;
            }

            @Override
            public Set<Subsystem> getRequirements() {
                return Collections.emptySet();
            }
        };
    }

    public Command rotateIndexerOuttake(int pos) {
        if (pos == 0) {
            return rotateIndexer(Indexer.POSITION_INDEXER_SERVO_SLOT_ZERO_OUTPUT);
        }
        else if (pos == 1) {
            return rotateIndexer(Indexer.POSITION_INDEXER_SERVO_SLOT_ONE_OUTPUT);
        }
        else if (pos == 2) {
            return rotateIndexer(Indexer.POSITION_INDEXER_SERVO_SLOT_TWO_OUTPUT);
        }
        else {
            throw new RuntimeException("Tried to create a Command to set the Indexer to an undefined position");
        }
    }

    public Command rotateIndexerIntake(int pos) {
        if (pos == 0) {
            return rotateIndexer(Indexer.POSITION_INDEXER_SERVO_SLOT_ZERO_INTAKE);
        }
        else if (pos == 1) {
            return rotateIndexer(Indexer.POSITION_INDEXER_SERVO_SLOT_ONE_INTAKE);
        }
        else if (pos == 2) {
            return rotateIndexer(Indexer.POSITION_INDEXER_SERVO_SLOT_TWO_INTAKE);
        }
        else {
            throw new RuntimeException("Tried to create a Command to set the Indexer to an undefined position");
        }
    }

    public Command setLauncherVelocity(double velocity) {
        return new Command() {
            @Override
            public void initialize() {
                robot.getLauncher().setLauncherVelocity(velocity);
            }

            double launcherVelocity;

            @Override
            public void execute() {
                launcherVelocity = robot.getLauncher().getLauncherVelocity();
            }

            @Override
            public boolean isFinished() {
                return Math.abs(velocity - launcherVelocity) <= 20;
            }

            @Override
            public Set<Subsystem> getRequirements() {
                return Collections.emptySet();
            }
        };
    }

    public Command setFarLauncherVelocity() {
        return setLauncherVelocity(1565);
    }

    public Command setCloseLauncherVelocity() {
        return setLauncherVelocity(100);
    }

    public Command stopLauncher() {
        return setLauncherVelocity(0);
    }


    public Command kickBall() {
        return new SequentialCommandGroup(
            new InstantCommand(() -> robot.getLauncher().kickBall()),
            new WaitCommand(200)
        );
    }

    public Command resetKicker() {
        return new SequentialCommandGroup(
            new InstantCommand(() -> robot.getLauncher().resetKicker()),
            new WaitCommand(125)
        );
    }

    public Command waitUntilBallInIndexerIntake() {
        return new WaitUntilCommand(() -> !robot.getIndexer().getPredictedColor(
            robot.getIndexer().colorSensorIntakeL.getNormalizedColors(),
            robot.getIndexer().colorSensorIntakeR.getNormalizedColors(),
            ((DistanceSensor) robot.getIndexer().colorSensorIntakeL).getDistance(DistanceUnit.CM),
            ((DistanceSensor) robot.getIndexer().colorSensorIntakeR).getDistance(DistanceUnit.CM)).equals(ArtifactColor.NONE))
            .withTimeout(4000);
    }

    public Command rotateTurretServo(double positionDeg) {
        return new Command() {
            @Override
            public Set<Subsystem> getRequirements() {
                return Collections.emptySet();
            }

            @Override
            public void initialize() {
            }

            @Override
            public void execute() {
                robot.getLauncher().setTurretRelativeAngle(positionDeg);
            }

            @Override
            public boolean isFinished() {
                return Math.abs(positionDeg - robot.getLauncher().getTurretDegrees()) < 5;
            }
        };
    }

    public Command getPickupSequence(int endSlot) {
        return new SequentialCommandGroup(
            startIntake(),
            rotateIndexerIntake(0),
            waitUntilBallInIndexerIntake(),
            rotateIndexerIntake(1),
            waitUntilBallInIndexerIntake(),
            rotateIndexerIntake(2),
            waitUntilBallInIndexerIntake(),
            rotateIndexerOuttake(endSlot)
        );
    }

    public Command getOuttakeSequence(ArtifactColor[] motif, int greenLocation) {

        if (motif == null) {
            motif = new ArtifactColor[] {ArtifactColor.GREEN, ArtifactColor.PURPLE, ArtifactColor.PURPLE};
        }

        int[] shootingOrder = new int[] {
            motif[0] == ArtifactColor.GREEN ? greenLocation : (greenLocation == 0 ? 1 : 0),
            motif[1] == ArtifactColor.GREEN ? greenLocation : (greenLocation == 1 ? 0 : 1),
            motif[2] == ArtifactColor.GREEN ? greenLocation : (greenLocation == 2 ? 1 : 2)
        };

        return new SequentialCommandGroup(
            new ParallelCommandGroup(
                setFarLauncherVelocity(),
                rotateIndexerOuttake(shootingOrder[0])
            ),
            kickBall(),
            resetKicker(),
            rotateIndexerOuttake(shootingOrder[1]),
            kickBall(),
            resetKicker(),
            rotateIndexerOuttake(shootingOrder[2]),
            kickBall(),
            resetKicker(),
            stopLauncher()
        );
    }
}
