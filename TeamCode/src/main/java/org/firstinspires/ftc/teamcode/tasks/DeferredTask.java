package org.firstinspires.ftc.teamcode.tasks;

import androidx.annotation.NonNull;

import java.util.function.Supplier;

/**
 * A task that defers building its inner task until init() is called at runtime.
 * Use this when the task depends on state that isn't known at construction time
 * (e.g., obelisk pattern from AprilTag detection).
 */
public class DeferredTask extends Task {

    Supplier<Task> taskSupplier;
    Task innerTask;

    public DeferredTask(Supplier<Task> taskSupplier) {
        this.taskSupplier = taskSupplier;
    }

    @Override
    public void init() {
        innerTask = taskSupplier.get();
        innerTask.init();
    }

    @Override
    public boolean run() {
        return innerTask.run();
    }

    @NonNull
    @Override
    public String toString() {
        return "Deferred Task: " + (innerTask != null ? innerTask.toString() : "not yet built");
    }
}
