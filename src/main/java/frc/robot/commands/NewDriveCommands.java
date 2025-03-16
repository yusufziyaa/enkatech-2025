package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.exterior_elevator.ExteriorElevator;
import frc.robot.subsystems.intake.Intake;

public class NewDriveCommands {
  public static Command Hangar(ExteriorElevator exterior, Arm arm, Intake intake) {
    return null;
  }

  public static Command TersHangar() {
    return null;
  }

  public static Command ScoreL4(ExteriorElevator exterior,Arm arm,Intake intake) {
    return new SequentialCommandGroup(new ParallelCommandGroup(
      exterior.NL4(),
      arm.NL4(),
      intake.DesiredL4()
    )/*add shoot */);
  }

  public static Command ScoreL2() {
    return null;
  }

  public static Command ScoreL3() {
    return null;
  }

  public static Command Zero() {
    return null;
  }
}
