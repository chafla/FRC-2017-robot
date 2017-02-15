package org.usfirst.frc.team852.robot;

public enum XButton {
    A(1),
    B(2),
    X(3),
    Y(4),
    LB(5),
    RB(6),
    BACK(7),
    START(8),
    LSTICK(9),
    RSTICK(10);
    private int value;

    XButton(int value) {
        this.value = value;
    }

    public int get() {
        return this.value;
    }
}
