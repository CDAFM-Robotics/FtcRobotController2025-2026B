package org.firstinspires.ftc.teamcode.tasks;

import androidx.annotation.NonNull;

public abstract class Task {

    abstract public void init();

    abstract public boolean run();

    @NonNull
    abstract public String toString();

    public Task append(Task toAppend) {
        return new SequentialTask(this, toAppend);
    }

    public Task with(Task runWith) {
        return new ParallelTask(this, runWith);
    }

    public Task addDeadline(Task deadline) {
        return new DeadlineTask(deadline, this);
    }

    public Task raceWith(Task race) {
        return new RaceTask(this, race);
    }
}
