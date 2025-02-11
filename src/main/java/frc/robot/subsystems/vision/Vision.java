// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.stream.Stream;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class Vision extends SubsystemBase {
  /** Creates a new Vision. */
  VisionIO io;

  VisionIO limelight;

  private final VisionInputsAutoLogged inputs = new VisionInputsAutoLogged();
  private AprilTagFieldLayout fieldLayout = Constants.fieldLayout;
  private PhotonPoseEstimator poseEstimatorL = null;
  private PhotonPoseEstimator poseEstimatorR = null;

  private List<Matrix<N3, N1>> curStdDevs = new ArrayList<Matrix<N3, N1>>();

  public Vision() {}

  public Vision(VisionIO io) {
    curStdDevs.add(Constants.kSingleTagStdDevs);
    curStdDevs.add(Constants.kSingleTagStdDevs);
    this.io = io;

    poseEstimatorL =
        new PhotonPoseEstimator(
            fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, Constants.robot2Camera1);
    poseEstimatorR =
        new PhotonPoseEstimator(
            fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, Constants.robot2Camera2);

    poseEstimatorL.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    poseEstimatorR.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
  }

  @Override
  public void periodic() {
    if (io != null) io.updateInputs(inputs);

    Logger.processInputs("Vision/AutoLogged", inputs);
  }

  public Optional<EstimatedRobotPose> getEstimatedGlobalPose(int index) {
    Optional<EstimatedRobotPose> visionEst = Optional.empty();

    List<PhotonPipelineResult> pipelineResults;
    PhotonPoseEstimator poseEstimator;
    if (index == 0) {
      pipelineResults = io.getPipelineL();
      poseEstimator = poseEstimatorL;
    } else {
      pipelineResults = io.getPipelineR();
      poseEstimator = poseEstimatorR;
    }

    for (var change : pipelineResults) {
      // if (!change.hasTargets() || change.getBestTarget().objDetectConf)
      visionEst = poseEstimator.update(change);
      updateEstimationStdDevs(visionEst, change.getTargets(), poseEstimator, index);
      //

      if (Robot.isSimulation() && visionEst.isPresent()) {
        Logger.recordOutput("Vision/PoseEstimate" + index, visionEst.get().estimatedPose);
      } else if (Robot.isSimulation()) {
        Logger.recordOutput(
            "Vision/PoseEstimate" + index, new Pose3d(-1, -1, -1, new Rotation3d()));
      }
    }
    return visionEst;
  }

  public List<PhotonTrackedTarget> getAllTargets() {
    return Stream.concat(
            io.getLatestResultL().getTargets().stream(),
            io.getLatestResultR().getTargets().stream())
        .toList();
  }

  public PhotonTrackedTarget getBiggestTarget() {
    double biggestArea = 0;
    PhotonTrackedTarget biggestAreaID = null;
    var targetN = getAllTargets();
    for (PhotonTrackedTarget photonTrackedTarget : targetN) {
      if (photonTrackedTarget.area > biggestArea) {
        biggestArea = photonTrackedTarget.area;
        biggestAreaID = photonTrackedTarget;
      }
    }
    return biggestAreaID;
  }

  public PhotonTrackedTarget getBestTargetL() {
    PhotonPipelineResult res = io.getLatestResultL();
    if (!res.hasTargets()) return null;
    return res.getBestTarget();
  }

  public PhotonTrackedTarget getBestTargetR() {
    PhotonPipelineResult res = io.getLatestResultR();
    if (!res.hasTargets()) return null;
    return res.getBestTarget();
  }

  // FIXME: BIG PROBLEM
  // because the standart deviations are calculated for both cameras, as their stddevs are equal.
  // this makes the odometry we get very unreliable
  //

  private void updateEstimationStdDevs(
      Optional<EstimatedRobotPose> estimatedPose,
      List<PhotonTrackedTarget> targets,
      PhotonPoseEstimator poseEstimator,
      int index) {
    if (estimatedPose.isEmpty()) {
      // No pose input. Default to single-tag std devs
      curStdDevs.set(index, Constants.kSingleTagStdDevs);

    } else {
      // Pose present. Start running Heuristic
      var estStdDevs = Constants.kSingleTagStdDevs;
      int numTags = 0;
      double avgDist = 0;

      // Precalculation - see how many tags we found, and calculate an
      // average-distance metric
      double minDist = 1e9;
      for (var tgt : targets) {
        var tagPose = poseEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
        if (tagPose.isEmpty()) continue;
        numTags++;
        double tagDist =
            tagPose
                .get()
                .toPose2d()
                .getTranslation()
                .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
        avgDist += tagDist;

        minDist = Double.min(minDist, tagDist);
      }

      if (numTags == 0) {
        // No tags visible. Default to single-tag std devs
        curStdDevs.set(index, Constants.kSingleTagStdDevs);
      } else {
        // One or more tags visible, run the full heuristic.
        avgDist /= numTags;
        // Decrease std devs if multiple targets are visible
        if (numTags > 2) estStdDevs = Constants.kMultiTagStdDevs;
        // Increase std devs based on (average) distance
        if (numTags < 3 || minDist > 3)
          estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        else {
          estStdDevs = estStdDevs.times((minDist * minDist / 30));
        }
        curStdDevs.set(index, estStdDevs);
      }
    }
  }

  public final PhotonTrackedTarget getTarget(int ID) {
    List<PhotonTrackedTarget> targets = getAllTargets();
    for (PhotonTrackedTarget photonTrackedTarget : targets) {
      if (photonTrackedTarget.getFiducialId() == ID) return photonTrackedTarget;
    }
    return null;
  }

  public double getLimelightYaw(int targetID) {
    return io.getLimelightYaw(targetID);
  }

  public Matrix<N3, N1> getEstimationStdDevs(int index) {
    return curStdDevs.get(index);
  }

  public void simulationPeriodic(Pose2d pose) {
    io.setRobotPose(pose);

    PhotonTrackedTarget target = getTarget(18);
    if (target == null) Logger.recordOutput("Odometry/BestTarget", (double) 0);
    else
      Logger.recordOutput(
          "Odometry/BestTarget",
          target.getDetectedCorners().get(0).y
              - target.getDetectedCorners().get(3).y
              - target.getDetectedCorners().get(1).y
              + target.getDetectedCorners().get(2).y);
  }
}
