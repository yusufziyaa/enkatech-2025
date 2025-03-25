// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : Mode.SIM;

  // kameranın robota göre konumu
  // TODO: fix camera position
  public static final Transform3d robot2CameraL =
      new Transform3d(0.305, 0, 0.2425, new Rotation3d(0, Math.toRadians(-5), Math.toRadians(0)));
  public static final Transform3d robot2CameraR =
      new Transform3d(0.305, 0, -0.2425, new Rotation3d(0, Math.toRadians(-5), Math.toRadians(0)));

  public static final Pose2d initialPose = new Pose2d(3, 3, new Rotation2d(0));

  public static final AprilTagFieldLayout fieldLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);

  public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(50, 50, 100);
  public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);

  // 0 ile 1 arasinda deger almali. ne kadar yuksek o kadar tolerans
  public static final double visionPoseEstimationMaxAmbiguity = 0.2;

  public static final double elevatorArmLength = 0.463;
  public static final double elevatorAngle = 70.2;

  public static final double elevatorMaxLength = 2;
  public static final double elevatorMinLength = 0.1;

  public static final Pose2d upperCoralPos = new Pose2d(0.8, 2.1, new Rotation2d());
  public static final Pose2d midCoralPos = new Pose2d(0.9, 1.3, new Rotation2d());
  public static final Pose2d lowerCoralPos = new Pose2d(0.75, 0.8, new Rotation2d());

  public static final Pose2d stationPos = new Pose2d(-0.11, 1.05, new Rotation2d());

  public static final APPROACH_TYPE upperApproachType = APPROACH_TYPE.LOWER;
  public static final APPROACH_TYPE midApproachType = APPROACH_TYPE.UPPER;
  public static final APPROACH_TYPE lowerApproachType = APPROACH_TYPE.UPPER;

  public static final CANcoderConfiguration initialCancoderConfig = new CANcoderConfiguration();

  public static int ShooterCANID = 31;
  public static int GripperCANID = 35;

  public static int ArmCANID = 32;
  public static int InteriorElevatorCANID = 38;

  public static int ExteriorElevatorCANID_AKU = 36;
  public static int ExteriorElevatorCANID_N = 37;

  public static int MZ80_1_ID = 8;
  public static int MZ80_2_ID = 9;
  public static double shooterAdjustingVoltage = 1;
  public static double shooterBackupVoltage = 1;
  public static double shootingVoltage = 4;

  public static int IntakeCANID = 8;

  public static int TirmanmaCANID = 40;
  public static double tirmanmaVoltage = 12;

  public static enum APPROACH_TYPE {
    LOWER,
    UPPER
  };

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  };
}
