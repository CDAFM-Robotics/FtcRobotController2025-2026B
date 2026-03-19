package org.firstinspires.ftc.teamcode.tasks;

import androidx.annotation.NonNull;

import java.util.function.Supplier;

public class WaitUntilTask extends Task {
    private Supplier<Boolean> toCheck;

    public WaitUntilTask (Supplier<Boolean> toCheck) {
        this.toCheck = toCheck;
    }

    @Override
    public void init() {

    }

    @Override
    public boolean run() {
        return toCheck.get();
    }

    @NonNull
    @Override
    public String toString() {
        return "Wait Until Action: " + toCheck.toString();
    }
}
