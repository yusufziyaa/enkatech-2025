package frc.robot.positions;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.exterior_elevator.ExteriorElevator;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.interior_elevator.InteriorElevator;

public class MechanismController {
  Intake intake;
  ExteriorElevator exteriorElevator;
  InteriorElevator interiorElevator;
  Arm arm;

  public MechanismController(
      Intake intake, ExteriorElevator exterior, InteriorElevator interior, Arm arm) {
    this.intake = intake;
    this.exteriorElevator = exterior;
    this.interiorElevator = interior;
    this.arm = arm;
  }

  private double getInteriorPosition() {
    return interiorElevator.getPosition();
  }

  private Command EmptyCommand = new InstantCommand(() -> {});

  private Command conditionalZero(InteriorElevator interior) {
    if (interior.getPosition() < 10) {
      return new SequentialCommandGroup(arm.asansorHareket(), interiorElevator.getToHigh());
    }
    return EmptyCommand;
  }

  public Command getToZero(InteriorElevator interior) {
    return new SequentialCommandGroup(
        exteriorElevator.getToGround(), conditionalZero(interior), arm.getToZero());
  }

  public Command getToHangar(InteriorElevator interior) {
    return new SequentialCommandGroup(
        exteriorElevator.getToGround(), conditionalZero(interior), arm.hangar2());
  }

  public Command getToL2(InteriorElevator interior) {
    return new SequentialCommandGroup(
        conditionalZero(interior), exteriorElevator.getToL2(), arm.getToL2L3());
  }
}
