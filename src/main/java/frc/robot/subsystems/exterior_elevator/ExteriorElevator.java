// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.exterior_elevator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import org.littletonrobotics.junction.Logger;

public class ExteriorElevator extends SubsystemBase {
  /** Creates a new ExteriorElevator. */
  ExteriorElevatorIO io;

  ExteriorElevatorIOInputsAutoLogged inputs = new ExteriorElevatorIOInputsAutoLogged();

  public ExteriorElevator(ExteriorElevatorIO io) {
    this.io = io;
  }

  double state = 0;

  public double getState() {
    return state;
  }

  public Command getToGround() {
    return new InstantCommand(
        () -> {
          state = 0;
          io.getToPosition(0); // should be 0, 5 for safety
        });
  }

  public Command getToL2() {
    return new InstantCommand(
        () -> {
          state = 1;
          io.getToPosition(28);
        });
  }

  public Command getToL3() {
    return new InstantCommand(
        () -> {
          state = 2;
          io.getToPosition(83);
        });
  }

  public Command getToL4() {
    return new InstantCommand(
        () -> {
          state = 3;
          io.getToPosition(93);
        });
  }

  public Command shootInCorrectAngle(Shooter shooter, Intake intake) {
    if (state == 3) {
      return intake.shootInCorrectAngle(1, shooter);
    } else return intake.shootInCorrectAngle(-1, shooter);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(getName(), inputs);
    // This method will be called once per scheduler run
  }
}
