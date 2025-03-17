package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.LimelightHelpers;
import java.util.Arrays;
import java.util.List;
import org.littletonrobotics.junction.Logger;
import org.photonvision.targeting.PhotonTrackedTarget;

public class AutoCommands {

  public static class Reef {
    Pose2d pose;
    int blueID;
    int redID;

    Reef(Pose2d pose, int blueID, int redID) {
      this.blueID = blueID;
      this.pose = pose;
      this.redID = redID;
    }

    int getID() {
      return DriverStation.getAlliance().orElse(Alliance.Red) == Alliance.Blue
          ? this.blueID
          : this.redID;
    }
  }

  public static Reef getNearest(Pose2d pose, List<Reef> reefs) {
    Reef minReef = null;
    double minDist = 1e9;
    for (Reef reef : reefs) {
      double dist =
          (reef.pose.getX() - pose.getX()) * (reef.pose.getX() - pose.getX())
              + (reef.pose.getY() - pose.getY()) * (reef.pose.getY() - pose.getY());
      dist = Math.sqrt(dist);
      if (dist <= minDist) {
        minReef = reef;
        minDist = dist;
      }
    }
    return minReef;
  }
  // lookat PIDs
  private static final PIDController lookAtTurnPID = new PIDController(0.01, 0.01, 0);

  // align PIDs
  // TODO: recalibrate turn
  private static final PIDController alignTurnPID = new PIDController(0.5, 0, 0);
  private static final PIDController alignDrivePID = new PIDController(0.2, 0, 0);

  // private static final PIDController drivePID = new PIDController(0.1, 0, 0);

  private static final PathConstraints constraints =
      new PathConstraints(
          3.79, 5.0, Units.degreesToRadians(540), Units.degreesToRadians(720), 12.6, false);

  private static final PathConstraints newconst = new PathConstraints(0.05, 0.03, 0.05, 0.03);

  public static List<Rotation2d> angles =
      Arrays.asList(
          new Rotation2d(),
          new Rotation2d(),
          new Rotation2d(),
          new Rotation2d(),
          new Rotation2d(),
          new Rotation2d(),
          new Rotation2d(),
          new Rotation2d(Units.degreesToRadians(180)),
          new Rotation2d(),
          new Rotation2d(),
          new Rotation2d(),
          new Rotation2d(),
          new Rotation2d(),
          new Rotation2d(),
          new Rotation2d(),
          new Rotation2d(),
          new Rotation2d(),
          new Rotation2d(),
          new Rotation2d());

  public static List<Reef> reefs =
      Arrays.asList(
          new Reef(new Pose2d(3.78, 2.83, new Rotation2d(Units.degreesToRadians(-120))), 17, 8),
          new Reef(new Pose2d(3.14, 4.02, new Rotation2d(Units.degreesToRadians(180))), 18, 7),
          new Reef(
              new Pose2d(3.81, 5.21, new Rotation2d(Units.degreesToRadians(120))), 19, 6), // 120
          new Reef(new Pose2d(5.21, 5.21, new Rotation2d(Units.degreesToRadians(60))), 20, 11),
          new Reef(new Pose2d(5.88, 4.02, new Rotation2d(Units.degreesToRadians(0))), 21, 10),
          new Reef(new Pose2d(5.16, 2.82, new Rotation2d(Units.degreesToRadians(-60))), 22, 9));

  public static Rotation2d getReefRotation(double id) {
    // System.out.println(id);
    for (Reef reef : reefs) {
      if (reef.redID == id || reef.blueID == id) {
        return reef.pose.getRotation();
      }
    }
    return new Rotation2d();
  }

  public static List<Pose2d> stations =
      Arrays.asList(
          new Pose2d(1.14, 6.92, new Rotation2d(Units.degreesToRadians(307.23))),
          new Pose2d(1.21, 1.06, new Rotation2d(Units.degreesToRadians(52.99))));

  public static List<Integer> reefIDSBlue = Arrays.asList(17, 18, 19, 20, 21, 22);

  public static List<Integer> reefIDSRed = Arrays.asList(8, 7, 6, 11, 10, 9);

  public static List<Integer> getReefIDS() {
    return DriverStation.getAlliance().orElse(Alliance.Red) == Alliance.Blue
        ? reefIDSBlue
        : reefIDSRed;
  }

  public static double getModuloRotation(double rawYaw) {
    double modified = (Math.abs(rawYaw) % (360)) * Math.signum(rawYaw);
    if (modified < -180) modified += 360;
    if (modified > 180) modified -= 360;
    return modified;
  }

  public static Command getPathfindingCommand(
      Drive drive, Pose2d targetPose, boolean invertRotation) {
    // FIXME
    // eğer invertRotation true olursa robot ters yaklaşıyor.
    // arka ve öne birer kamera takmak lazım
    Pose2d targetPoseNew =
        new Pose2d(drive.getPose().getX(), drive.getPose().getY(), drive.getPose().getRotation());
    ;
    if (invertRotation)
      targetPoseNew =
          new Pose2d(
              targetPose.getX(),
              targetPose.getY(),
              new Rotation2d(Math.PI + targetPose.getRotation().getRadians()));
    return AutoBuilder.pathfindToPose(targetPoseNew, constraints);
  }

  public static final Command getToStationA(Drive drive) {
    return getPathfindingCommand(drive, stations.get(0), false);
  }

  public static final Command getToStationB(Drive drive) {
    return getPathfindingCommand(drive, stations.get(1), false);
  }

  public static Pair<Command, Integer> getToClosestReef(Drive drive, boolean inverseApproach) {
    Reef nearest = getNearest(drive.getPose(), reefs);
    Command getTo = getPathfindingCommand(drive, nearest.pose, inverseApproach);
    getTo.addRequirements(drive);
    return new Pair<Command, Integer>(getTo, nearest.getID());
  }

  public static Command getToReef(Drive drive, int reefNumber, boolean inverseApproach) {
    Command getTo = getPathfindingCommand(drive, reefs.get(reefNumber).pose, inverseApproach);
    return getTo;
  }

  static PIDController pid;
  static PIDController pidX;
  static PIDController pidY;

  /*public static Command betterAlign(Vision vision,Drive drive){
    Transform3d robot2Camera = LimelightHelpers.get
  }*/

  public static Command turnToReef(Vision vision, Drive drive) {
    Pose3d pose = LimelightHelpers.getTargetPose3d_CameraSpace("limelight");
    double desiredAngle = pose.getRotation().getY();
    Logger.recordOutput("desired", desiredAngle);
    ;
    PIDController controller = new PIDController(0.01, 0, 0);
    return new FunctionalCommand(
        () -> {},
        () -> {
          double angleerror =
              -(drive.getPose().getRotation().getRadians() - desiredAngle) * 360 / (Math.PI * 2);
          // Logger.recordOutput("", null);
          if (angleerror > 180) angleerror -= 360;
          if (angleerror < -180) angleerror += 360;
          Logger.recordOutput("error-angle", angleerror);

          ChassisSpeeds speeds = new ChassisSpeeds(0, 0, controller.calculate(angleerror));
          // drive.runVelocity(speeds);
        },
        (Boolean cons) -> {
          ChassisSpeeds speeds = new ChassisSpeeds(0, 0, 0);
          // drive.runVelocity(speeds);
        },
        () -> {
          return false;
        },
        vision);
  }

  public static Command alignFree(Vision vision, Drive drive) {
    return alignToReef(vision, drive, 1);
  }

  public static Command alignL2L3(Vision vision, Drive drive) {
    return alignToReef(vision, drive, 0.67);
  }

  public static Command alignL4(Vision vision, Drive drive) {
    return alignToReef(vision, drive, 0.2);
  }

  public static Command alignBall(Vision vision, Drive drive) {
    return alignToReef(vision, drive, 0.42);
  }

  public static Command alignToReef(Vision vision, Drive drive, double dUzaklik) {
    // System.out.println(LimelightHelpers.getFiducialID("limelight"));
    // Rotation2d reefrotation = angles.get((int) LimelightHelpers.getFiducialID("limelight"));
    // System.out.println(reefrotation);
    pid = new PIDController(0.02, 0, 0);
    pidX = new PIDController(0.6, 0, 0);
    pidY = new PIDController(0.08, 0, 0);
    return new SequentialCommandGroup(
        vision.setOrient(false),
        new FunctionalCommand(
            () -> {},
            () -> {
              if (LimelightHelpers.getFiducialID("limelight") == -1) return;
              vision.orient = true;
              // Pose3d tag2Limelight = LimelightHelpers.getTargetPose3d_RobotSpace("limelight");
              Rotation2d reefrotation =
                  getReefRotation((int) LimelightHelpers.getFiducialID("limelight"));
              Pose3d tag2Robot = LimelightHelpers.getTargetPose3d_CameraSpace("limelight");
              // System.out.println(reefrotation.getDegrees());
              // System.out.println(drive.getPose().getRotation().getDegrees());
              double turningError =
                  reefrotation.getDegrees()
                      - getModuloRotation(drive.getRawGyroRotation().getDegrees());
              if (turningError > 180) turningError -= 360;
              if (turningError < -180) turningError += 360;
              Logger.recordOutput("errorangle", turningError);
              // Logger.recordOutput("transform-l", tag2Robot);
              // Logger.recordOutput("reefrotaton", reefrotation.getDegrees());
              // Logger.recordOutput("limelighttransform", tag2Limelight);

              double uzaklik =
                  Math.sqrt(
                      tag2Robot.getY() * tag2Robot.getY() + tag2Robot.getZ() * tag2Robot.getZ());
              double saghareket = uzaklik * pidY.calculate(LimelightHelpers.getTX("limelight") + 3);

              double ileriHareket = 0;
              if (uzaklik < dUzaklik || Math.abs(uzaklik - dUzaklik) < 0.1) {
                ileriHareket = (uzaklik - dUzaklik) * 2;
              } else ileriHareket = 1.2;

              if (Math.abs(saghareket) > 0.3) saghareket = 0.25 * Math.signum(saghareket);
              Logger.recordOutput("uzaklik", uzaklik);
              if (Math.abs(saghareket) < 0.01) saghareket = 0;

              ChassisSpeeds speeds =
                  new ChassisSpeeds(
                      -pidX.calculate(ileriHareket), saghareket, -pid.calculate(turningError));
              // hope this works (no reason not to, just doesnt)
              Logger.recordOutput("speeds", speeds);
              drive.runVelocity(speeds);
            },
            (Boolean cons) -> {
              ChassisSpeeds speeds = new ChassisSpeeds(0, 0, 0);
              drive.runVelocity(speeds);
            },
            () -> {
              Pose3d tag2Robot = LimelightHelpers.getTargetPose3d_CameraSpace("limelight");
              double uzaklik =
                  Math.sqrt(
                      tag2Robot.getY() * tag2Robot.getY() + tag2Robot.getZ() * tag2Robot.getZ());
              Rotation2d reefrotation =
                  getReefRotation(LimelightHelpers.getFiducialID("limelight"));
              double turningError =
                  reefrotation.getDegrees()
                      - getModuloRotation(drive.getRawGyroRotation().getDegrees());
              if (turningError > 180) turningError -= 360;
              if (turningError < -180) turningError += 360;
              if ((Math.abs(uzaklik - dUzaklik) < 0.01 || uzaklik < 0.22)
                  && Math.abs(turningError) < 1
                  && (LimelightHelpers.getTX("limelight") + 3.5) < 1) return true;
              if (LimelightHelpers.getFiducialID("limelight") == -1) return true;
              return false;
            },
            vision,
            drive));
  }

  static PIDController drivePIDX;
  static PIDController drivePIDY;
  static PIDController angularPID;

  public static Command alignToReefMultiCam(Vision vision, Drive drive) {
    drivePIDX = new PIDController(0.05, 0.1, 0);
    drivePIDY = new PIDController(0.05, 0.1, 0);
    angularPID = new PIDController(2, 0, 0);
    return new FunctionalCommand(
        () -> {},
        () -> {
          PhotonTrackedTarget target = vision.getBestTargetL();
          if (target == null) return;
          Transform3d camera2Target = target.getBestCameraToTarget();
          Logger.recordOutput("degs", camera2Target.getRotation().getZ());
          ChassisSpeeds speeds =
              new ChassisSpeeds(
                  -drivePIDX.calculate(camera2Target.getX() - 0.36),
                  -drivePIDY.calculate(camera2Target.getY() + 0.11),
                  0 * -angularPID.calculate(camera2Target.getRotation().getZ() + 2.74));
          drive.runVelocity(speeds);
        },
        (Boolean cons) -> {
          ChassisSpeeds speeds = new ChassisSpeeds(0, 0, 0);
          drive.runVelocity(speeds);
        },
        () -> {
          return vision.getBestTargetL() == null;
        },
        vision,
        drive);
  }

  public static Command alignToCurrentReef(Vision vision, Drive drive) {
    Reef reef;
    reef = getNearest(drive.getPose(), reefs);
    final FunctionalCommand command =
        new FunctionalCommand(
            () -> {},
            () -> {
              double targetYaw = vision.getBiggestTarget().yaw;
              double angularSpeed = alignTurnPID.calculate(targetYaw);
              double ySpeed = alignDrivePID.calculate(targetYaw);

              // TODO: try real gyro and incorporate turning to the right angle if possible

              ChassisSpeeds speeds = new ChassisSpeeds(0, ySpeed, angularSpeed);
              drive.runVelocity(speeds);
            },
            drive::stopConsumer,
            () -> {
              if (vision.getLimelightYaw() < 1) {
                return true;
              }
              return false;
            },
            vision,
            drive);

    return command;
  }

  public static Command lookAtCurrentReef(Vision vision, Drive drive) {
    Integer id;
    id = getNearest(drive.getPose(), reefs).getID();

    final FunctionalCommand command =
        new FunctionalCommand(
            () -> {},
            () -> {
              double angularSpeed = 0, ySpeed = 0;
              var target = vision.getTarget(id);
              if (target != null) {
                angularSpeed =
                    lookAtTurnPID.calculate(drive.getMaxAngularSpeedRadPerSec() * target.getYaw());
                // ySpeed = (Math.PI - Math.abs(modSpeed)) * Math.signum(modSpeed);

                // System.out.println(Math.sin(ySpeed));
              }
              ChassisSpeeds speeds =
                  new ChassisSpeeds(
                      0,
                      ySpeed,
                      angularSpeed); // positive is counter clockwise, speeds are relative to the
              // robot
              drive.runVelocity(speeds);
            },
            drive::stopConsumer,
            () -> {
              PhotonTrackedTarget target = vision.getTarget(id);
              if (target != null && Math.abs(target.getYaw()) < 0.05) {
                drive.stop();
                return true;
              }
              return false;
            },
            drive,
            vision);
    command.addRequirements(vision, drive);

    return command;
  }
}
