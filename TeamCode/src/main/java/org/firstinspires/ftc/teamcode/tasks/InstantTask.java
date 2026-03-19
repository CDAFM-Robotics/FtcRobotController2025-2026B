package org.firstinspires.ftc.teamcode.tasks;


import androidx.annotation.NonNull;

import java.util.function.Supplier;

public class InstantTask extends Task{

    Runnable onInit;

    public InstantTask(Runnable init) {
        onInit = init;
    }

    @Override
    public void init() {
        onInit.run();
    }

    @Override
    public boolean run() {
        return true;
    }

    @NonNull
    @Override
    public String toString() {
        return "Instant Task: " + onInit.toString();
    }
}
