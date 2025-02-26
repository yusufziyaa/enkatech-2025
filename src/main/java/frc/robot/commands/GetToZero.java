// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.exterior_elevator.ExteriorElevator;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.interior_elevator.InteriorElevator;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class GetToZero extends SequentialCommandGroup {
  /** Creates a new GetToZero. */
  public GetToZero(Intake intake, Arm arm, InteriorElevator interior, ExteriorElevator exterior) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    if (interior.getPosition() < 10) {
      addCommands(
          exterior.getToGround(), arm.asansorHareket(), interior.getToHigh(), arm.getToZero());
    } else {
      addCommands(exterior.getToGround(), arm.getToZero());
    }
  }
}
