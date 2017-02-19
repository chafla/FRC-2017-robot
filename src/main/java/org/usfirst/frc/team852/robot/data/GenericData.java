package org.usfirst.frc.team852.robot.data;

import java.util.concurrent.atomic.AtomicBoolean;

public class GenericData {

    private AtomicBoolean invalid = new AtomicBoolean(false);
    private DataType dataType;

    public GenericData(final DataType dataType) {
        this.dataType = dataType;
    }

    public boolean isInvalid() {
        return this.invalid.get();
    }

    public void setInvalid() {
        System.out.println(this.dataType.getUpdateMsg());
        this.invalid.set(true);
    }

    public String getAlreadyReadMsg() {
        return this.dataType.getAlreadyReadMsg();
    }

    public String getTimedOutMsg() {
        return this.dataType.getTimedOutMsg();
    }
}
