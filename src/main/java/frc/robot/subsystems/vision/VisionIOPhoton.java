package frc.robot.subsystems.vision;

import frc.robot.util.LimelightHelpers;
import java.util.List;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

public class VisionIOPhoton implements VisionIO {
  public PhotonCamera cameraL;
  public PhotonCamera cameraR;
  String limelight;

  public VisionIOPhoton(String camera_1_name, String camera_2_name, String limelight_name) {
    cameraL = new PhotonCamera(camera_1_name);
    cameraR = new PhotonCamera(camera_2_name);
    limelight = limelight_name;
  }

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
  public List<PhotonPipelineResult> getPipelineR() {
    return cameraR.getAllUnreadResults();
  }

  @SuppressWarnings("removal")
  @Override
  public PhotonPipelineResult getLatestResultL() {
    return cameraL.getLatestResult();
  }

  @SuppressWarnings("removal")
  @Override
  public PhotonPipelineResult getLatestResultR() {
    return cameraR.getLatestResult();
  }

  @Override
  public double getLimelightYaw(int targetID) {
    return LimelightHelpers.getTX(limelight);
  }
}
