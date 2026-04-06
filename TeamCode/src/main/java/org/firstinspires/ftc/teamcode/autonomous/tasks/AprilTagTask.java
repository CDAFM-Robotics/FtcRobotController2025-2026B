package org.firstinspires.ftc.teamcode.autonomous.tasks;

import androidx.annotation.NonNull;

import org.firstinspires.ftc.teamcode.common.Robot;
import org.firstinspires.ftc.teamcode.common.RobotStaticValuesClass;
import org.firstinspires.ftc.teamcode.tasks.Task;

public class AprilTagTask extends Task {
    Robot robot;
    boolean isRed;

    public AprilTagTask(Robot robot, boolean isRed) {
        this.robot = robot;
        this.isRed = isRed;
    }

    @Override
    public void init() {
        robot.getLauncher().getLimeLight().pipelineSwitch(7);
    }

    RobotStaticValuesClass.Obelisk motif = null;

    @Override
    public boolean run() {
        motif = robot.getLauncher().getObelisk(isRed);
        return motif != null;
    }

    public RobotStaticValuesClass.Obelisk getMotif() {
        return motif;
    }

    @NonNull
    @Override
    public String toString() {
        return "AprilTagTask";
    }
}
