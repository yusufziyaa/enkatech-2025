package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import java.util.List;
import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;

public class VisionIOSim implements VisionIO {
  private final VisionSystemSim visionSystemSim = new VisionSystemSim("main");
  private PhotonCameraSim sim;

  private final PhotonCamera camera = new PhotonCamera("camera");
  private AprilTagFieldLayout fieldLayout = Constants.fieldLayout;

  public VisionIOSim() {

    // https://docs.photonvision.org/en/latest/docs/simulation/simulation-java.html
    visionSystemSim.addAprilTags(fieldLayout);

    SimCameraProperties properties = new SimCameraProperties();
    properties.setAvgLatencyMs(35);
    properties.setCalibError(0.25, 0.08);
    properties.setLatencyStdDevMs(5);

    properties.setFPS(30);
    properties.setCalibration(640, 480, Rotation2d.fromDegrees(100));

    // camera = new PhotonCamera("camera");

    sim = new PhotonCameraSim(camera, properties);
    visionSystemSim.addCamera(sim, Constants.robot2Camera);

    sim.enableProcessedStream(true);
    sim.enableRawStream(true);
    sim.enableDrawWireframe(true);
  }

  @Override
  public void setRobotPose(Pose2d pose) {
    visionSystemSim.update(pose);
    SmartDashboard.putData("vision_field", visionSystemSim.getDebugField());
    // Logger.recordOutput("FieldSimulation/VisionField", visionSystemSim.getDebugField());
  }

  @Override
  public void updateInputs(VisionInputs inputs) {
    inputs.connected = true;
  }

  @Override
  public List<PhotonPipelineResult> getPipeline() {
    return camera.getAllUnreadResults();
  }
}
