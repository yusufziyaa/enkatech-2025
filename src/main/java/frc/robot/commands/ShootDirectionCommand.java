// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.exterior_elevator.ExteriorElevator;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ShootDirectionCommand extends Command {
  /** Creates a new ShootDirectionCommand. */
  Shooter m_Shooter;

  Intake m_Intake;
  ExteriorElevator mExterior;

  double exteriorState = 0;
  double desiredVal = 0;

  public ShootDirectionCommand(Shooter shooter, Intake intake, ExteriorElevator exteriorElevator) {
    m_Shooter = shooter;
    m_Intake = intake;
    mExterior = exteriorElevator;

    addRequirements(shooter);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("state", mExterior.getState());
    m_Shooter.runVoltage(
        Constants.shootingVoltage
            * (mExterior.getState() == 3 ? -1 : 1)
            * (m_Intake.getDesired() == 0 ? 1 : -1));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Shooter.runVoltage(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
