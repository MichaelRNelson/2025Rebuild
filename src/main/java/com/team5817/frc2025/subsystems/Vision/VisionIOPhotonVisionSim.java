package com.team5817.frc2025.subsystems.Vision;

import static com.team5817.frc2025.field.FieldLayout.kTagMap;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;

import java.util.function.Supplier;

import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import edu.wpi.first.math.geometry.Rotation2d;

public class VisionIOPhotonVisionSim implements VisionDeviceIO {

  private static VisionSystemSim visionSim;

  private final Supplier<Pose2d> poseSupplier;
  private final PhotonCamera camera;
  private final PhotonCameraSim cameraSim;
  private final String name;

  public VisionIOPhotonVisionSim(String name, Transform3d robotToCamera, Supplier<Pose2d> poseSupplier) {
    this.name = name;
    this.poseSupplier = poseSupplier;

    // Initialize actual camera instance used by sim
    this.camera = new PhotonCamera(name);

    if (visionSim == null) {
      visionSim = new VisionSystemSim("main");
      visionSim.addAprilTags(kTagMap);
    }

    SimCameraProperties properties = new SimCameraProperties();
    properties.setCalibration(960, 720, Rotation2d.fromDegrees(90));
    properties.setFPS(30);
    properties.setAvgLatencyMs(30);

    this.cameraSim = new PhotonCameraSim(camera, properties);
    visionSim.addCamera(cameraSim, robotToCamera);
  }

//do i even need this??

  // public void updateInputs() {
  //   // update PhotonVision sim with current robot pose
  //   visionSim.update(poseSupplier.get());

  //   var result = camera.getLatestResult();

  //   if (result.hasTargets()) {
  //     var best = result.getBestTarget();

  //     result.getMultiTagResult().estimatedPose.ifPresent(estimated -> {
  //       VisionFrame frame = new VisionFrame(
  //         name,
  //         estimated.getTimestampSeconds(),
  //         estimated.getPose().toPose2d(),
  //         best.getFiducialId(),
  //         best.getYaw(),
  //         best.getPitch()
  //       );

  //       VisionDeviceManager.addVisionFrame(frame);
  //     });
  //   }
  // }
}
