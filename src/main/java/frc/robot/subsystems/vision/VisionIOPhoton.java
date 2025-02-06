package frc.robot.subsystems.vision;

import frc.robot.util.LimelightHelpers;
import java.util.List;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

public class VisionIOPhoton implements VisionIO {
  PhotonCamera cameraL = new PhotonCamera("camera_left");
  PhotonCamera cameraR = new PhotonCamera("camera_right");

  // TODO: not tested yet
  public VisionIOPhoton() {}

  @Override
  public void updateInputs(VisionInputs inputs) {

    inputs.connectedLeft = cameraL.isConnected();
    inputs.ledModeLeft = cameraL.getLEDMode();

    inputs.connectedRight = cameraR.isConnected();
    inputs.ledModeRight = cameraR.getLEDMode();
  }
  // FIXME
  @Override
  public List<PhotonPipelineResult> getPipelineL() {
    return cameraL.getAllUnreadResults();
  }

  @Override
  public double getLimelightYaw(int targetID) {
    return LimelightHelpers.getTX("limelight");
  }
}
