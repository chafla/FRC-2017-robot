package org.usfirst.frc.team852.robot.data;

public class HeadingData extends GenericData {
    private final double degree;

    public HeadingData(double degree) {
        this.degree = degree;
    }

    public double getDegree() {
        return this.degree;
    }
}
