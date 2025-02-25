package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import java.util.List;
import org.littletonrobotics.junction.AutoLog;
import org.photonvision.PhotonCamera;
import org.photonvision.common.hardware.VisionLEDMode;
import org.photonvision.targeting.PhotonPipelineResult;

public interface VisionIO {
  public Transform3d robot2Camera = new Transform3d();

  public PhotonCamera cameraL = new PhotonCamera("camera1");
  public PhotonCamera cameraR = new PhotonCamera("camera2");

  default void setRobotPose(Pose2d pose) {}

  default void updateInputs(VisionInputs inputs) {}

  default PhotonPipelineResult getLatestResultL() {
    return null;
  }

  default PhotonPipelineResult getLatestResultR() {
    return null;
  }

  default List<PhotonPipelineResult> getPipelineL() {
    return null;
  }

  default List<PhotonPipelineResult> getPipelineR() {
    return null;
  }

  default double getLimelightYaw(int targetID) {
    return 0;
  }

  @AutoLog
  public static class VisionInputs {
    boolean connectedLeft;
    boolean connectedRight;

    VisionLEDMode ledModeLeft;
    VisionLEDMode ledModeRight;
  }
}
