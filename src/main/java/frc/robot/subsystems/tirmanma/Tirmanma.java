// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.tirmanma;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Tirmanma extends SubsystemBase {
  /** Creates a new Tirmanma. */
  TirmanmaIO io;

  TirmanmaIOInputsAutoLogged inputs = new TirmanmaIOInputsAutoLogged();

  public Tirmanma(TirmanmaIO io) {
    this.io = io;
  }

  public Tirmanma() {}

  public Command runAtVoltage(double nVoltage) {
    return new InstantCommand(
        () -> {
          io.runVoltage(nVoltage);
        },
        this);
  }

  public Command tirman() {
    return runToPosition(480); // acma konumu 110
  }

  public Command runToPosition(double position) {
    return new FunctionalCommand(
        () -> {},
        () -> {
          if (Math.abs(position - inputs.position) > 20) {
            io.runVoltage(Constants.tirmanmaVoltage);
          } else io.runVoltage((position - inputs.position) / 4);
        },
        (Boolean cons) -> {
          io.runVoltage(0);
        },
        () -> {
          return position - inputs.position < 0;
        },
        this);
  }

  public void runVoltage(double v) {
    io.runVoltage(v);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    org.littletonrobotics.junction.Logger.processInputs(getName(), inputs);
    // This method will be called once per scheduler run
  }
}
