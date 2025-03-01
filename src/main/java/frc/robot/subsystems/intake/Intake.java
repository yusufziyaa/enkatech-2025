// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.shooter.Shooter;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  IntakeIO io;

  double deltaPerRotation = 2 * Math.PI;

  IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  public Intake(IntakeIO io) {
    this.io = io;
  }

  double desired = 0;

  public double getDesired() {
    return desired;
  }

  public void setDesiredToLeft() {
    desired = -36.6;
  }

  public void setDesiredToRight() {
    desired = 0;
  }

  public void toggleDesired() {
    if (desired == 0) desired = -36.6;
    else desired = 0;
  }

  public Command changeToDesired() {
    return new InstantCommand(
        () -> {
          io.runToPosition(desired);
        });
  }

  public Command waitTillCenter() {
    return new FunctionalCommand(
        () -> {
          io.runToPosition(-18.3);
        },
        () -> {},
        (Boolean cons) -> {},
        () -> {
          return Math.abs(inputs.position + 18.3) < 1;
        },
        this);
  }

  public Command adjustToCenter() {
    return new InstantCommand(
        () -> {
          io.runToPosition(-18.3);
        },
        this);
  }

  public Command adjustToLeft() {
    return new InstantCommand(
        () -> {
          io.runToPosition(-36.6);
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

  public Command shootInCorrectAngle(double adj, Shooter shooter) {
    double d = adj * (desired == 0 ? 1 : -1);
    if (d == 1) return shooter.shootLeft();
    else return shooter.shootRight();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(getName(), inputs);
    // This method will be called once per scheduler run
  }
}
