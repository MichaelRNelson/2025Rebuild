package com.team5817.frc2025;

import com.team5817.frc2025.subsystems.Drive.Drive;
import com.team5817.frc2025.subsystems.Elevator.Elevator;
import com.team5817.frc2025.subsystems.EndEffector.EndEffector;
import com.team5817.frc2025.subsystems.Intake.Intake;
import com.team5817.frc2025.subsystems.Intake.IntakeConstants;
import com.team5817.frc2025.subsystems.Vision.VisionDeviceManager;
import com.team5817.frc2025.subsystems.rollers.RollerSystemIOTalonFX;

public class RobotContainer {
    private Drive mDrive;
    private Elevator mElevator;
    private EndEffector mEndEffector;
    private Intake mIntake;
    private VisionDeviceManager mVision;

    public RobotContainer(){
        mIntake = new Intake(
            new RollerSystemIOTalonFX(Ports.INTAKE_ROLLER, IntakeConstants., null, 0)
        );
    }
}
