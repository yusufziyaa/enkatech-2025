package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import java.util.List;
import org.littletonrobotics.junction.AutoLog;
import org.photonvision.common.hardware.VisionLEDMode;
import org.photonvision.targeting.PhotonPipelineResult;

public interface VisionIO {
  public Transform3d robot2Camera = new Transform3d();

  default void setRobotPose(Pose2d pose) {}

  default void updateInputs(VisionInputs inputs) {}

  default PhotonPipelineResult getLatestResult() {
    return null;
  }

  default List<PhotonPipelineResult> getPipeline() {
    return null;
  }

  @AutoLog
  public static class VisionInputs {
    public boolean connected = false;
    public VisionLEDMode ledMode = VisionLEDMode.kOff;
  }
}
