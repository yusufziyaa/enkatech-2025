package frc.robot.positions;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.exterior_elevator.ExteriorElevator;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.interior_elevator.InteriorElevator;
import org.littletonrobotics.junction.Logger;

public class HangarPosition implements Position {
  Intake intake;
  ExteriorElevator exterior;
  InteriorElevator interior;
  Arm arm;

  public HangarPosition(
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
    Logger.recordOutput("denemeeee", 123);
    return new SequentialCommandGroup(exterior.getToGround(), arm.instantZero());
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
  }

  @Override
  public Command getToL4() {
    return new SequentialCommandGroup(exterior.getToL4(), arm.getToL4());
  }

  @Override
  public Command getToGround() {
    return new SequentialCommandGroup(interior.getToLow(), arm.runVoltageZero());
  }
}
