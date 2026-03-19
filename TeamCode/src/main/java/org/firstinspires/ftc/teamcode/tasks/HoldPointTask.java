package org.firstinspires.ftc.teamcode.tasks;

import androidx.annotation.NonNull;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

public class HoldPointTask extends Task {
    Follower follower;
    Pose holdPose;
    public HoldPointTask(Follower follower, Pose pose) {
        this.follower = follower;
        holdPose = pose;
    }

    @Override
    public void init() {
        follower.holdPoint(holdPose);
    }

    @Override
    public boolean run() {
        return true;
    }

    @NonNull
    @Override
    public String toString() {
        return "Hold Point Task: " + holdPose.toString();
    }
}
