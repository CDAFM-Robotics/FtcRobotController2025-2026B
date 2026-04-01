package org.firstinspires.ftc.teamcode.tasks;

import androidx.annotation.NonNull;

import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.util.RobotLog;

public class FollowPathTask extends Task{
    Follower follower;
    PathChain path;
    double speed;

    public FollowPathTask(Follower follower, PathChain pathChain) {
        this.follower = follower;
        this.path = pathChain;
        speed = 1;
    }

    public FollowPathTask(Follower follower, PathChain pathChain, double speed) {
        this.follower = follower;
        this.path = pathChain;
        this.speed = speed;
    }

    @Override
    public void init() {
        follower.followPath(path, speed, false);
    }

    @Override
    public boolean run() {
        RobotLog.d("Robot Position: x: %.3f, y: %.3f, heading: %.3f", follower.getPose().getX(), follower.getPose().getY(), follower.getHeading());
        RobotLog.d("Target Position: x: %.3f, y: %.3f, heading: %.3f", path.endPose().getX(), path.endPose().getY(), path.endPose().getHeading());
        return !follower.isBusy();
    }

    @NonNull
    @Override
    public String toString() {
        return "Follow Path Task: " + path.toString();
    }


}
