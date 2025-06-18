package com.team5817.frc2025;

import com.team5817.frc2025.subsystems.Superstructure;
import com.team5817.frc2025.subsystems.Drive.Drive;
import com.team5817.frc2025.subsystems.Elevator.Elevator;
import com.team5817.frc2025.subsystems.Elevator.ElevatorConstants;
import com.team5817.frc2025.subsystems.EndEffector.EndEffectorConstants;
import com.team5817.frc2025.subsystems.EndEffector.EndEffectorConstants.EndEffectorWristConstants;
import com.team5817.frc2025.subsystems.EndEffector.EndEffectorWrist;
import com.team5817.frc2025.subsystems.Intake.Intake;
import com.team5817.frc2025.subsystems.Intake.IntakeConstants;
import com.team5817.frc2025.subsystems.Rollers.RollerSubsystem;
import com.team5817.frc2025.subsystems.Rollers.RollerSubsystemIOSim;
import com.team5817.frc2025.subsystems.Rollers.RollerSubsystemIOTalonFX;
import com.team5817.frc2025.subsystems.Vision.VisionDeviceManager;
import com.team5817.lib.drivers.ServoMotorIOSim;
import com.team5817.lib.drivers.ServoMotorIOTalonFX;

import edu.wpi.first.math.system.plant.DCMotor;

public class RobotContainer {
  public Drive mDrive;
  public Elevator mElevator;
  public EndEffectorWrist mEndEffectorWrist;
  public RollerSubsystem<EndEffectorConstants.RollerState> mEndEffectorRollers;
  public Intake mIntake;
  public VisionDeviceManager mVision;
  public Superstructure mSuperstructure;

  public RobotContainer() {
    if (Robot.isReal())
      makeRealRobot();
    else
      makeSimulatedRobot();

    SubsystemManager mSubsystemManager = SubsystemManager.getInstance();

    mSuperstructure = new Superstructure(
        mDrive,
        mElevator,
        mEndEffectorWrist,
        mEndEffectorRollers,
        mIntake);

    mSubsystemManager.setSubsystems(
        mDrive,
        mSuperstructure,
        mVision,
        mElevator,
        mEndEffectorRollers,
        mEndEffectorWrist,
        mIntake);

  }

  public void makeRealRobot() {
    mEndEffectorWrist = new EndEffectorWrist(
        new ServoMotorIOTalonFX(EndEffectorWristConstants.kWristServoConstants));

    mEndEffectorRollers = new RollerSubsystem<EndEffectorConstants.RollerState>(
        EndEffectorConstants.RollerState.IDLE,
        "EndEffectorRollers",
        new RollerSubsystemIOTalonFX(
            Ports.ENDEFFECTOR_ROLLER,
            EndEffectorConstants.rollerConstants,
            1));

    mIntake = new Intake(
        new RollerSubsystemIOTalonFX(Ports.INTAKE_ROLLER, IntakeConstants.RollerConstants.motorConstants, 1),
        new RollerSubsystemIOTalonFX(Ports.SIDE_INDEXER, IntakeConstants.RollerConstants.motorConstants, 1),
        new RollerSubsystemIOTalonFX(Ports.BOTTOM_INDEXER, IntakeConstants.RollerConstants.motorConstants, 1),
        new ServoMotorIOTalonFX(ElevatorConstants.kElevatorServoConstants));
    mElevator = new Elevator(
        new ServoMotorIOTalonFX(ElevatorConstants.kElevatorServoConstants));
    mDrive = new Drive();
    mVision = VisionDeviceManager.getInstance();
  }

  public void makeSimulatedRobot() {
    mEndEffectorWrist = new EndEffectorWrist(
        new ServoMotorIOSim(EndEffectorConstants.EndEffectorWristConstants.kWristServoConstants));
    mEndEffectorRollers = new RollerSubsystem<EndEffectorConstants.RollerState>(
        EndEffectorConstants.RollerState.IDLE,
        "EndEffectorRollers",
        new RollerSubsystemIOSim(DCMotor.getKrakenX60(1), 1, .01));

    mIntake = new Intake(
        new RollerSubsystemIOSim(DCMotor.getKrakenX60(1), 1, 0.01),
        new RollerSubsystemIOSim(DCMotor.getKrakenX60(1), 1, 0.01),
        new RollerSubsystemIOSim(DCMotor.getKrakenX60(1), 1, 0.01),
        new ServoMotorIOSim(IntakeConstants.DeployConstants.kDeployServoConstants));
    mElevator = new Elevator(
        new ServoMotorIOSim(ElevatorConstants.kElevatorServoConstants));
    mDrive = new Drive();
    mVision = VisionDeviceManager.getInstance();
  }
}
