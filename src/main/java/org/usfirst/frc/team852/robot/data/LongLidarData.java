package org.usfirst.frc.team852.robot.data;

public class LongLidarData extends GenericData {
    private final int cm;

    public LongLidarData(int cm) {
        this.cm = cm;
    }

    public int getCm() {
        return this.cm;
    }
}