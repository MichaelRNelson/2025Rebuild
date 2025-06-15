package com.team5817.frc2025.subsystems.rollers;

import com.team5817.frc2025.subsystems.rollers.RollerSystem.RollerControlMode;

public interface RollerState {
    public double getDemand();
    public RollerControlMode getControlMode();
}
