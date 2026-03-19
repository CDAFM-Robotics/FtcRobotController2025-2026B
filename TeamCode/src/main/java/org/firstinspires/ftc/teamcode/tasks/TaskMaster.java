package org.firstinspires.ftc.teamcode.tasks;

import java.util.function.Supplier;

public class TaskMaster {
    private Task task = new NullTask();
    private boolean taskInited = false;

    public TaskMaster() {
    }

    public TaskMaster(Task task) {
        this.task = task;
    }

    public void addTask(Task task) {
        if (task instanceof NullTask) {
            taskInited = false;
        }
        task = new SequentialTask(this.task, task);
    }

    boolean status;

    public void update() {
        if (!taskInited) {
            task.init();
            taskInited = true;
        }

        status = task.run();
        if (status && !(task instanceof NullTask)) {
            task = new NullTask();
        }
    }

    public boolean getStatus() {
        return status;
    }

    public Task getTask() {
        return task;
    }
}
