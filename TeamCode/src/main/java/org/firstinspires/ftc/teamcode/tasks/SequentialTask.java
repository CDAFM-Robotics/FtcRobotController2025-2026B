package org.firstinspires.ftc.teamcode.tasks;

import androidx.annotation.NonNull;

public class SequentialTask extends Task {
    Task[] tasks;
    int currentTask = 0;
    public SequentialTask(Task... tasks) {
        this.tasks = tasks;
    }

    @Override
    public void init() {
        tasks[0].init();
    }

    @Override
    public boolean run() {
        if (tasks[currentTask].run()) {
            currentTask++;
            if (currentTask >= tasks.length) {
                return true;
            }
            tasks[currentTask].init();
        }
        return false;
    }

    @NonNull
    @Override
    public String toString() {
        StringBuilder toReturn = new StringBuilder();
        toReturn.append("Sequential Task {\n");
        boolean firstLoop = true;
        for (Task task : tasks) {
            if (!firstLoop) {
                toReturn.append(", ");
            }
            firstLoop = false;
            toReturn.append(task.toString());
        }
        return toReturn + "\n}";
    }
}
