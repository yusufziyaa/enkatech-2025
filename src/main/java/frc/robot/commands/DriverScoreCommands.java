package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.exterior_elevator.ExteriorElevator;
import frc.robot.subsystems.gripper.Gripper;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.interior_elevator.InteriorElevator;
import java.util.function.DoubleSupplier;

public class DriverScoreCommands {

  public static SequentialCommandGroup ZeroPOS(
      InteriorElevator interior, ExteriorElevator exterior, Intake intake, Arm arm) {
    return new SequentialCommandGroup(
        exterior.getToGround(), intake.adjustToCenter(), arm.asansorHareket());
  }

  public static SequentialCommandGroup Hangar(
      InteriorElevator interior, ExteriorElevator exterior, Intake intake, Arm arm) {
    return new SequentialCommandGroup(
        exterior.getToGround(), intake.adjustToCenter(), arm.hangar(), interior.getToLow());
  }

  public static SequentialCommandGroup scoreL3(
      InteriorElevator interiorElevator, ExteriorElevator exterior, Arm arm, Intake intake) {
    return new SequentialCommandGroup(
        new ParallelCommandGroup(arm.asansorHareket()),
        new ParallelCommandGroup(interiorElevator.getToHigh(), intake.adjustToCenter()),
        exterior.getToL3(),
        arm.getToL2L3(),
        intake.adjustToRight());
  }

  public static SequentialCommandGroup scoreL2(
      InteriorElevator interiorElevator, ExteriorElevator exterior, Arm arm, Intake intake) {
    return new SequentialCommandGroup(
        new ParallelCommandGroup(arm.asansorHareket()),
        new ParallelCommandGroup(interiorElevator.getToHigh(), intake.adjustToCenter()),
        exterior.getToL2(),
        arm.getToL2L3(),
        intake.adjustToRight());
  }

  public static SequentialCommandGroup zeroToL3(Intake intake, Arm arm, ExteriorElevator exterior) {
    return new SequentialCommandGroup(
        exterior.getToL3(), arm.getToL2L3(), intake.changeToDesired());
  }

  public static SequentialCommandGroup zeroToL2(Intake intake, Arm arm, ExteriorElevator exterior) {
    return new SequentialCommandGroup(
        exterior.getToL2(), arm.getToL2L3(), intake.changeToDesired());
  }

  public static SequentialCommandGroup zeroToL4(Intake intake, Arm arm, ExteriorElevator exterior) {
    return new SequentialCommandGroup(exterior.getToL4(), arm.getToL4(), intake.changeToDesired());
  }

  public static SequentialCommandGroup zeroToStart(
      Intake intake, Arm arm, ExteriorElevator exterior, InteriorElevator interior) {
    return new SequentialCommandGroup(
        exterior.getToGround(), arm.asansorHareket(), interior.getToLow());
  }

  public static SequentialCommandGroup zeroToGround(
      Intake intake, ExteriorElevator exterior, Arm arm, InteriorElevator interior) {
    return new SequentialCommandGroup(interior.getToLow());
  }

  public static SequentialCommandGroup startToZero(Arm arm, InteriorElevator interior) {
    return new SequentialCommandGroup(arm.asansorHareket(), interior.getToHigh(), arm.getToZero());
  }

  public static Command retreatL4(
      ExteriorElevator exterior, InteriorElevator interior, Arm arm, Intake intake) {
    return new ParallelCommandGroup(
        intake.adjustToCenter(), interior.getToHigh(), arm.getToZero(), exterior.getToGround());
  }

  public static Command gripperControl(Gripper gripper, DoubleSupplier voltage) {
    return new RunCommand(
        () -> {
          gripper.setVoltage(voltage.getAsDouble());
        },
        gripper);
  }
}
