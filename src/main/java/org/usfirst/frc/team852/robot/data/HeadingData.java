package org.usfirst.frc.team852.robot.data;

/**
 * Created by jonathanvictorino on 2/14/17.
 */
public class HeadingData extends GenericData {
    private final double degree;

    public HeadingData(double degree) {
        this.degree = degree;
    }

    public double getDegree() {
        return this.degree;
    }
}
