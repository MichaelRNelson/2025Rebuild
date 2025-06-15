package com.team5817.frc2025.subsystems.Intake;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.team5817.frc2025.Ports;
import com.team5817.frc2025.RobotConstants;
import com.team5817.frc2025.subsystems.rollers.TalonFXRollerConstants;
import com.team5817.frc2025.subsystems.rollers.RollerState;
import com.team5817.frc2025.subsystems.rollers.RollerSystem.RollerControlMode;
import com.team5817.lib.drivers.ServoMotorSubsystem.ServoMotorSubsystemConstants;
import com.team5817.lib.drivers.ServoMotorSubsystemWithCancoder.AbsoluteEncoderConstants;

import lombok.Getter;
public class IntakeConstants {
	/**
	 * Constants related to the Intake Deploy subsystem.
	 */
	public static final class DeployConstants {
		public static final ServoMotorSubsystemConstants kDeployServoConstants = new ServoMotorSubsystemConstants();

		public static final AbsoluteEncoderConstants kDeployEncoderConstants = new AbsoluteEncoderConstants();

		static {

			kDeployServoConstants.kName = "Deploy";
			kDeployServoConstants.simIO = RobotConstants.isComp? false:true;

			kDeployServoConstants.kMainConstants.id = Ports.INTAKE_PIVOT;
			kDeployServoConstants.kMainConstants.counterClockwisePositive = false;
			

			kDeployServoConstants.kHomePosition = 0; // degrees
			kDeployServoConstants.kRotationsPerUnitDistance = (1.0 / 360.0)*20;

			kDeployServoConstants.kMaxUnitsLimit = 100000;
			kDeployServoConstants.kMinUnitsLimit = -100000;

			kDeployServoConstants.kKp = 3.8125	;
			kDeployServoConstants.kKi = 0.0;   
			kDeployServoConstants.kKd = 0;
			kDeployServoConstants.kKa = 0;
			kDeployServoConstants.kKs = 0;
			kDeployServoConstants.kKv = .5;
			kDeployServoConstants.kKg = 0.265625;

 
			kDeployServoConstants.kCruiseVelocity = 32*360*1/20; 
			kDeployServoConstants.kAcceleration = 32*360*1/20; 

			kDeployServoConstants.kMaxForwardOutput = 12.0;
			kDeployServoConstants.kMaxReverseOutput = -12.0;

			kDeployServoConstants.kEnableSupplyCurrentLimit = true;
			kDeployServoConstants.kSupplyCurrentLimit = 80; // amps
			kDeployServoConstants.kSupplyCurrentThreshold = 80; // amps
			kDeployServoConstants.kSupplyCurrentTimeout = 0.01; // seconds

			kDeployServoConstants.kEnableStatorCurrentLimit = true;
			kDeployServoConstants.kStatorCurrentLimit = 80; // amps

			kDeployServoConstants.kNeutralMode = NeutralModeValue.Brake;

			kDeployEncoderConstants.rotor_to_sensor_ratio = 20;
			kDeployEncoderConstants.remote_encoder_port = Ports.INTAKE_CANCODER;

			kDeployServoConstants.kHomingOutput = -.3;
			kDeployServoConstants.kHomingTimeout = 0.2;
			kDeployServoConstants.kHomingVelocityWindow = 5;
		}

	}

	public static final class MotorConstants{
		TalonFXRollerConstants feederConstants = new TalonFXRollerConstants();
		feederConstants.kKp = 0;
	}

	public static final class IntakeStates{
		public enum FeederState implements RollerState{
			IDLE(0),
			INTAKING(10),
			EXHAUST(-6);
			
			@Getter private final double demand;
			@Getter private final RollerControlMode controlMode;
			
			FeederState(double demand){
				this.demand = demand;
				this.controlMode = RollerControlMode.VOLTAGE;
			}
		}
	
		public enum SideIndexerState implements RollerState{
			IDLE(0),
			INTAKING(8),
			EXHAUST(-6),
			SLOW_EXAUST(-2);
			@Getter private final double demand;
			@Getter private final RollerControlMode controlMode;
			
			SideIndexerState(double demand){
				this.demand = demand;
				this.controlMode = RollerControlMode.VOLTAGE;
			}
		}
	
		public enum LowIndexerState implements RollerState{
			IDLE(0),
			INTAKING(2.5),
			EXHAUST(-6),
			SLOW_EXAUST(-2);
			@Getter private final double demand;
			@Getter private final RollerControlMode controlMode;
			
			LowIndexerState(double demand){
				this.demand = demand;
				this.controlMode = RollerControlMode.VOLTAGE;
			}
		}
	}
}
