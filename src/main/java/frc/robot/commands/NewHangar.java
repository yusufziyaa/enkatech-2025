// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.exterior_elevator.ExteriorElevator;
import frc.robot.subsystems.gripper.Gripper;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.vision.HangarVision;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class NewHangar extends ParallelCommandGroup {
  /** Creates a new NewHangar. */
  public NewHangar(
      ExteriorElevator exterior,
      Intake intake,
      Arm arm,
      HangarVision vision,
      Drive drive,
      Gripper gripper,
      Shooter shooter) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        AutoCommands.alignToHangar(vision, drive),
        NewDriveCommands.Hangar(exterior, arm, intake),
        gripper.gripTillSeen(shooter));
  }
}
