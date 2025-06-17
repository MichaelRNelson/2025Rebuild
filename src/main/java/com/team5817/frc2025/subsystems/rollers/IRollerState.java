package com.team5817.frc2025.subsystems.rollers;

import com.team5817.frc2025.subsystems.rollers.RollerSubsystem.RollerControlMode;

public interface IRollerState {
  public double getDemand();

  public RollerControlMode getControlMode();
}
