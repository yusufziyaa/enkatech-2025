package frc.robot.positions;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.exterior_elevator.ExteriorElevator;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.interior_elevator.InteriorElevator;

public class ZeroPosition implements Position {
  Intake intake;
  ExteriorElevator exterior;
  InteriorElevator interior;
  Arm arm;

  public ZeroPosition(
      Intake intake,
      Arm arm,
      InteriorElevator interiorElevator,
      ExteriorElevator exteriorElevator) {
    this.intake = intake;
    this.exterior = exteriorElevator;
    this.interior = interiorElevator;
    this.arm = arm;
  }

  @Override
  public Command getToZero() {
    return new InstantCommand(() -> {});
  }

  @Override
  public Command getToL2() {
    return new SequentialCommandGroup(exterior.getToL2(), arm.getToL2L3());
  }

  @Override
  public Command getToL3() {
    return new SequentialCommandGroup(exterior.getToL3(), arm.getToL2L3());
  }

  @Override
  public Command getToHangar() {
    return new InstantCommand(
        () -> {
          arm.instantHangar2();
        });
    // return new SequentialCommandGroup(arm.instantHangar2());
  }

  @Override
  public Command getToGround() {
    return new SequentialCommandGroup(
        new ParallelCommandGroup(arm.getToNull(), new WaitCommand(0.5)),
        interior.getToLow(),
        arm.runVoltageZero());
  }

  @Override
  public Command getToL4() {
    return new SequentialCommandGroup(arm.getToL4(), exterior.getToL4());
  }
}
