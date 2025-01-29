package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.Vision;
import java.util.Arrays;
import java.util.List;
import org.photonvision.targeting.PhotonTrackedTarget;

public class AutoCommands {

  private static final PathConstraints constraints =
      new PathConstraints(
          3.79, 5, Units.degreesToRadians(540), Units.degreesToRadians(720), 12.6, false);

  // HER RESIF ICIN X KOORDINATI FARKLI OLMALI
  // FIXME
  public static List<Pose2d> reefPositions =
      Arrays.asList(
          new Pose2d(3.65, 2.73, new Rotation2d(Units.degreesToRadians(-300))),
          new Pose2d(3, 4, new Rotation2d()),
          new Pose2d(3.67, 5.41, new Rotation2d(Units.degreesToRadians(-60))),
          new Pose2d(5.36, 5.44, new Rotation2d(Units.degreesToRadians(-120))),
          new Pose2d(6.18, 4, new Rotation2d(Units.degreesToRadians(-180))),
          new Pose2d(5.4, 2.66, new Rotation2d(Units.degreesToRadians(-240))));

  public static List<Integer> reefIDSBlue = Arrays.asList(17, 18, 19, 20, 21, 22);

  public static List<Integer> reefIDSRed = Arrays.asList(8, 7, 6, 11, 10, 9);

  public static List<Integer> getReefIDS() {
    return DriverStation.getAlliance().orElse(Alliance.Red) == Alliance.Blue
        ? reefIDSBlue
        : reefIDSRed;
  }

  public static Command getPathfindingCommand(Pose2d targetPose) {
    // FIXME
    return AutoBuilder.pathfindToPose(targetPose, constraints, 0);
  }

  public static Pair<Command, Integer> getToClosestReef(Drive drive) {
    Pose2d nearest = drive.getPose().nearest(reefPositions);
    Command getTo = getPathfindingCommand(drive.getPose().nearest(reefPositions));
    Integer id = -1;
    for (int i = 0; i < 6; i++) {
      if (nearest.getX() == reefPositions.get(i).getX()) id = getReefIDS().get(i);
    }
    getTo.addRequirements(drive);
    return new Pair<Command, Integer>(getTo, id);
  }

  public static Command getToReef(int reefNumber) {
    Command getTo = getPathfindingCommand(reefPositions.get(reefNumber));
    return getTo;
  }

  public static Command alignToCurrentReef(Vision vision, Drive drive) {
    double pGain = 10;

    Integer id;
    Command getToPosition;

    Pair<Command, Integer> pair = getToClosestReef(drive);
    getToPosition = pair.getFirst();
    id = pair.getSecond();

    final Command command =
        Commands.run(
            () -> {
              double speed = 0;
              var target = vision.getAllTargets();
              for (PhotonTrackedTarget photonTrackedTarget : target) {
                if (photonTrackedTarget.getFiducialId() == id) {
                  speed =
                      -1
                          * pGain
                          * drive.getMaxAngularSpeedRadPerSec()
                          * photonTrackedTarget.getYaw();
                }
              }
              System.out.println(drive.getMaxAngularSpeedRadPerSec());

              ChassisSpeeds speeds = new ChassisSpeeds(0, 0, speed);
              drive.runVelocity(speeds);
            },
            drive,
            vision);

    command.addRequirements(vision, drive);
    return getToPosition.andThen(command);
  }
}
