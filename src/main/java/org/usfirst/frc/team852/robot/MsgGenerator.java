package org.usfirst.frc.team852.robot;

@FunctionalInterface
public interface MsgGenerator {
    String getMesssage(Robot robot, String desc);
}
