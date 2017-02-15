package org.usfirst.frc.team852.robot.strategies;

import org.usfirst.frc.team852.robot.Robot;

public interface Strategy {

    void xboxAButtonPressed(Robot robot);

    void xboxBButtonPressed(Robot robot);

    void xboxXButtonPressed(Robot robot);

    void xboxYButtonPressed(Robot robot);
}
