package org.firstinspires.ftc.teamcode.tasks;

import androidx.annotation.NonNull;

public class SleepTask extends Task {

    private long delay;

    public SleepTask(long msDelay) {
        delay = msDelay;
    }

    private long initTime;
    @Override
    public void init() {
        initTime = System.currentTimeMillis();
    }

    @Override
    public boolean run() {
        return System.currentTimeMillis() - initTime >= delay;
    }

    @NonNull
    @Override
    public String toString() {
        return "Sleep Task: " + delay;
    }
}
