package org.usfirst.frc.team852.robot.data;

import java.util.concurrent.atomic.AtomicBoolean;

public class GenericData {

    private final long timestamp = System.currentTimeMillis();

    private AtomicBoolean dataRead = new AtomicBoolean(false);

    public long getTimestamp() {
        return timestamp;
    }

    public boolean isDataRead() {
        return dataRead.get();
    }

    public void setDataRead() {
        this.dataRead.set(true);
    }
}
