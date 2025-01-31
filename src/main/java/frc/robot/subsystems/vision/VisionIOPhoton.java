package frc.robot.subsystems.vision;

import java.util.List;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

public class VisionIOPhoton implements VisionIO {
  PhotonCamera camera = new PhotonCamera("camera");

  // TODO: not tested yet
  public VisionIOPhoton() {}

  @Override
  public void updateInputs(VisionInputs inputs) {
    inputs.connected = camera.isConnected();
    inputs.ledMode = camera.getLEDMode();
  }
  // FIXME
  @Override
  public List<PhotonPipelineResult> getPipelineL() {
    return camera.getAllUnreadResults();
  }
}
