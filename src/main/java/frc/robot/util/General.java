package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;

public class General {
  public static double DistancePose2d(Pose2d a, Pose2d b) {
    return Math.sqrt(
        (a.getX() - b.getX()) * (a.getX() - b.getX())
            + (a.getY() - b.getY()) * (a.getY() - b.getY()));
  }
}
