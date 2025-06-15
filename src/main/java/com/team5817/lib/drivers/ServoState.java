package com.team5817.lib.drivers;


    public interface ServoState {
        double getDemand();
        boolean isDisabled();
        double getAllowableError();
        ServoMotorSubsystem.ControlState getControlState();
    }