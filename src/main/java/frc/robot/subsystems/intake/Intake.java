// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  IntakeIO io;

  double deltaPerRotation = 2 * Math.PI;

  IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  public Intake(IntakeIO io) {
    this.io = io;
  }

  public Command adjustToCenter() {
    return new InstantCommand(
        () -> {
          io.runToPosition(deltaPerRotation * 2);
        },
        this);
  }

  public Command adjustToLeft() {
    return new InstantCommand(
        () -> {
          io.runToPosition(deltaPerRotation * 4);
        },
        this);
  }

  public Command adjustToRight() {
    return new InstantCommand(
        () -> {
          io.runToPosition(0);
        },
        this);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(getName(), inputs);
    // This method will be called once per scheduler run
  }
}
