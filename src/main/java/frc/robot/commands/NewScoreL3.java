// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.exterior_elevator.ExteriorElevator;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.LimelightHelpers;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class NewScoreL3 extends SequentialCommandGroup {
  /** Creates a new NewScoreL3. */
  public NewScoreL3(
      Vision vision,
      Drive drive,
      ExteriorElevator exterior,
      Arm arm,
      Intake intake,
      Shooter shooter) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new ConditionalCommand(
            new SequentialCommandGroup(
                new ParallelCommandGroup(
                    NewDriveCommands.ScoreL3(exterior, arm, intake),
                    AutoCommands.alignL3(vision, drive)),
                new WaitCommand(0.2),
                shooter.shootIfAvab(intake, exterior)),
            NewDriveCommands.ScoreL2(exterior, arm, intake),
            () -> {
              return LimelightHelpers.getFiducialID("limelight") != -1;
            }));
  }
}
