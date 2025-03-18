// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class HangarVision extends SubsystemBase {
  /** Creates a new HangarVision. */
  PhotonCamera camera;
  // 1 sol 2 sağ
  // 13 sol 12 sağ
  public HangarVision(String camera_name) {
    camera = new PhotonCamera(camera_name);
  }

  @SuppressWarnings("deprecated")
  public Optional<PhotonTrackedTarget> getHangarTarget() {
    PhotonPipelineResult res = camera.getLatestResult();

    for (PhotonTrackedTarget tag : res.getTargets()) {
      if (tag.getFiducialId() == 1
          || tag.getFiducialId() == 2
          || tag.getFiducialId() == 13
          || tag.getFiducialId() == 12) return Optional.of(tag);
    }
    return Optional.empty();
  }

  @Override
  public void periodic() {
    Optional<PhotonTrackedTarget> tt = getHangarTarget();
    if (tt.isPresent()) {
      Logger.recordOutput("hangar_c2t", tt.get().getBestCameraToTarget());
      Logger.recordOutput("hangar_yaw", tt.get().getYaw());
    } else {
      Logger.recordOutput("hangar_c2t", new Transform3d());
      Logger.recordOutput("hangar_yaw", 0);
    }
    // This method will be called once per scheduler run
  }
}
