package org.firstinspires.ftc.teamcode.common.util;

public class Selection {
    /**
     * The range of the selection is from [0, range) (parentheses is non-inclusive)
     */

    int range;
    int value;

    public Selection(int range) {
        this.range = range;
        this.value = 0;
    }

    public void increment() {
        if (range == 0) {
            return;
        }

        value++;
        if (value >= range) {
            value = 0;
        }
    }

    public void decrement() {
        if (range == 0) {
            return;
        }

        value--;
        if (value < 0) {
            value = range - 1;
        }
    }

    public int getValue() {
        return value;
    }

    public int getRange() {
        return range;
    }

    public void setRange(int range) {
        this.range = range;
        if (value >= range) {
            value = range;
        }
    }


    public boolean getBoolValue() {
        return value != 0;
    }
}
