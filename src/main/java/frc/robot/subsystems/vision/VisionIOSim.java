package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import java.util.List;
import org.photonvision.PhotonCamera;
import org.photonvision.common.hardware.VisionLEDMode;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class VisionIOSim implements VisionIO {
  private final VisionSystemSim visionSystemSim = new VisionSystemSim("main");
  private PhotonCameraSim simL;
  private PhotonCameraSim simR;
  private PhotonCameraSim simF;

  private final PhotonCamera cameraL = new PhotonCamera("camera_sim_left");
  private final PhotonCamera cameraR = new PhotonCamera("camera_sim_right");
  private final PhotonCamera cameraF = new PhotonCamera("camera_sim_front");

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

    simL = new PhotonCameraSim(cameraL, properties);
    simR = new PhotonCameraSim(cameraR, properties);

    visionSystemSim.addCamera(simL, Constants.robot2CameraAKU);
    visionSystemSim.addCamera(simR, Constants.robot2CameraN);

    // sim.enableProcessedStream(true);
    // sim.enableRawStream(true);
    // sim.enableDrawWireframe(true);
  }

  @Override
  public void setRobotPose(Pose2d pose) {
    visionSystemSim.update(pose);
    SmartDashboard.putData("vision_field", visionSystemSim.getDebugField());

    // NetworkTableInstance.getDefault().getTable("limelight").getEntry

    // NetworkTableInstance.getDefault().getTable("limelight").putValue("tx", );

    // Logger.recordOutput("FieldSimulation/VisionField", visionSystemSim.getDebugField());
  }

  @SuppressWarnings("removal")
  @Override
  public double getLimelightYaw(int targetID) {
    for (PhotonTrackedTarget target : cameraF.getLatestResult().getTargets()) {
      if (target.getFiducialId() == targetID) {
        return target.getYaw();
      }
    }
    return 0;
  }

  @Override
  public void updateInputs(VisionInputs inputs) {
    inputs.connectedLeft = true;
    inputs.ledModeLeft = VisionLEDMode.kDefault;

    inputs.connectedRight = true;
    inputs.ledModeRight = VisionLEDMode.kDefault;
  }

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
}
