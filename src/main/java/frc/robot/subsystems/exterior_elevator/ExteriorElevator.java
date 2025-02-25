// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.exterior_elevator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class ExteriorElevator extends SubsystemBase {
  /** Creates a new ExteriorElevator. */
  ExteriorElevatorIO io;

  ExteriorElevatorIOInputsAutoLogged inputs = new ExteriorElevatorIOInputsAutoLogged();

  public ExteriorElevator(ExteriorElevatorIO io) {
    this.io = io;
  }

  public Command getToGround() {
    return new InstantCommand(
        () -> {
          io.getToPosition(0); // should be 0, 5 for safety
        });
  }

  public Command getToL2() {
    return new InstantCommand(
        () -> {
          io.getToPosition(28);
        });
  }

  public Command getToL3() {
    return new InstantCommand(
        () -> {
          io.getToPosition(83);
        });
  }

  public Command getToL4() {
    return new InstantCommand(
        () -> {
          io.getToPosition(93);
        });
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(getName(), inputs);
    // This method will be called once per scheduler run
  }
}
