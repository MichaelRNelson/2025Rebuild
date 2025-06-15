package com.team5817.frc2025.subsystems.Vision;

import com.team5817.frc2025.RobotState.VisionUpdate;
import com.team5817.frc2025.subsystems.Vision.LimelightHelpers.PoseEstimate;
import com.team5817.lib.drivers.Pigeon;
import com.team254.lib.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import java.util.Optional;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

/**
 * VisionDevice class handles vision processing and updates.
 */
public class VisionDevice {
	private VisionDeviceIOAutoLogged mPeriodicIO = new VisionDeviceIOAutoLogged();
	public Optional<VisionUpdate> visionUpdate = Optional.empty();

	public String mName;
	private NetworkTable mOutputTable;

	/**
	 * Constructor for VisionDevice.
	 *
	 * @param name the name of the vision device
	 */
	public VisionDevice(String name) {
		this.mName = name;
		mOutputTable = NetworkTableInstance.getDefault().getTable(name);


	}

	/**
	 * Updates the vision device with the latest data.
	 *
	 * @param timestamp the current timestamp
	 */
	public void update(double timestamp) {

		PoseEstimate poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(mName);
		mPeriodicIO.seesTarget = false;
		if(poseEstimate!=null)
			mPeriodicIO.seesTarget = poseEstimate.pose.getTranslation().getNorm() != 0 && LimelightHelpers.getTV(mName);
		if (mPeriodicIO.seesTarget) {
			final double realTime = timestamp - mPeriodicIO.latency;
			mPeriodicIO.fps = mOutputTable.getEntry("fps").getInteger(0);
			mPeriodicIO.latency = mOutputTable.getEntry("latency").getDouble(0.0);
			mPeriodicIO.tagId = mOutputTable.getEntry("tid").getNumber(-1).intValue();

			mPeriodicIO.mt1Pose = new Pose2d(LimelightHelpers.getBotPose2d_wpiBlue(mName));
			mPeriodicIO.targetToCamera = LimelightHelpers.getTargetPose3d_CameraSpace(mName);
			mPeriodicIO.tagCounts = poseEstimate.tagCount;
			mPeriodicIO.mt2Pose = new Pose2d(poseEstimate.pose);

			VisionUpdate visionUpdate = new VisionUpdate(mPeriodicIO.tagId, realTime, mPeriodicIO.ta,
					mPeriodicIO.mt2Pose.getTranslation());
					Logger.recordOutput(mName+"/ID", mPeriodicIO.tagId);
			this.visionUpdate = Optional.of(visionUpdate);

		} else {
			this.visionUpdate = Optional.empty();
		}
		LimelightHelpers.SetRobotOrientation(mName, Pigeon.getInstance().getYaw().getDegrees(), 0, 0, 0, 0, 0);
		Logger.recordOutput(mName+"/mt1", mPeriodicIO.mt1Pose.wpi());
	}



	/**
	 * Gets the latest vision update.
	 *
	 * @return an Optional containing the latest VisionUpdate, or empty if no update
	 *         is available
	 */
	public Optional<VisionUpdate> getVisionUpdate() {
		return this.visionUpdate;
	}

	/**
	 * Outputs telemetry data.
	 */
	public void outputTelemetry() {
	}

	/**
	 * Inner class to hold periodic IO data for the vision device.
	 */
	@AutoLog
	public static class VisionDeviceIO {

		// inputs

		double camera_exposure = 20;
		boolean camera_auto_exposure = false;
		double camera_gain = 10;

		// Outputs
		public long hb = 0;
		public long fps = -1;
		public boolean is_connected = true;
		public double latency = 0;
		public int tagId = 0;
		public boolean seesTarget = true;
		public double ta = 0;
		public boolean useVision = true;
		public double tagCounts = 0;
		public Pose2d mt2Pose = new Pose2d();
		public Pose2d mt1Pose = new Pose2d();
		public Pose3d targetToCamera = new Pose3d();
	}
}
