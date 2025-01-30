package frc.robot.commands;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Constants;
import frc.robot.subsystems.elevator.Elevator;

public class ElevatorCommands {

  // asansor yuksekligi (x) icin:
  /*
   * x^2 + 2x(a*cosA - bsinA) + (a^2 + b^2 - l^2) = 0 olmalı
   * a = asansörün kök noktası ile robot kolun geleceği nokta arasındaki yatay
   * uzaklık
   * b = üstteki ama dikey uzaklık
   * A = asansörün yer ile yaptığı açı (72.048 derece)
   * l = robot kolun uzunluğu
   *
   *
   * tabii ki delta = 4*(acosA - bsinA)^2 - 4a^2 - 4b^2 + 4l^2 >= 0 olmalı
   *
   * bu durumda robot kolun yer ile arasında yaptığı açı B = arccos((a+xcosA) / l)
   * = arcsin((b - xsinA) / l)
   * robot kolun asansör ile yaptığı açı Teta = A+B (önden açı)
   *
   */
  private static final double elevatorAngle = Constants.elevatorAngle; // angle between elevator and normal
  private static final double armLength = Constants.elevatorArmLength; // m, length of arm

  /*
   * asansöre göre verilen bir konum ile o konuma intake'in ulaşması için gereken
   * asansör yüksekliği ve kol açısı
   */
  public static Pair<Double, Double> getRequiredValues(Pose2d relativePose) {
    double a = 1;
    double b = 2
        * (relativePose.getX() * Math.cos(Math.toRadians(elevatorAngle))
            - relativePose.getY() * Math.sin(Math.toRadians(elevatorAngle)));
    double c = relativePose.getX() * relativePose.getX()
        + relativePose.getY() * relativePose.getY()
        - armLength * armLength;

    double delta = b * b - 4 * a * c;

    if (delta < 0)
      return null;

    double elevatorLength1 = (-b + Math.sqrt(delta)) / 2 * a;
    double elevatorLength2 = (-b - Math.sqrt(delta)) / 2 * a;

    double armAngle1 = 180
        - Math.toDegrees(
            Math.acos((elevatorLength1 * Math.sin(Math.toRadians(elevatorAngle)) - relativePose.getY()) / armLength))
        + 90 - elevatorAngle;

    double armAngle2 = 180
        - Math.toDegrees(
            Math.acos((elevatorLength2 * Math.sin(Math.toRadians(elevatorAngle)) - relativePose.getY()) / armLength))
        + 90 - elevatorAngle;

    return new Pair<Double, Double>(elevatorLength1, armAngle1);
  }

  public static Command adjustTo(
      Elevator elevator, Pose2d relativePose) { // relativePose of IK target to the robot, in meters

    elevator.io.addTargetMech(
        new Pose2d(-relativePose.getX(), relativePose.getY(), new Rotation2d()));

    Pair<Double,Double> ans = getRequiredValues(relativePose);

    if (ans==null) {
      //cant reach pos
      return null;
    }

    double usedElevatorLength = ans.getFirst();
    double armAngle = ans.getSecond();

    return adjustElevatorSetpoints(elevator, usedElevatorLength, armAngle, 0);
  }

  public static Command adjustElevatorSetpoints(
      Elevator elevator, double elevatorLength, double armAngle, double intakeAngle) {
    Command cmd = new RunCommand(
        () -> {
          /// System.out.println(elevatorLength-elevator.getElevatorEncoder());
          elevator.runVelocity(
              elevatorLength - elevator.getElevatorEncoder(),
              armAngle - elevator.getArmEncoder(),
              intakeAngle - elevator.getIntakeEncoder());
        });
    cmd.addRequirements(elevator);
    return cmd;
  }
}
