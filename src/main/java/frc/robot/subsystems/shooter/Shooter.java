// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  ShooterIO io;
  ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

  public Shooter() {}

  public Shooter(ShooterIO io) {
    this.io = io;
  }

  public Command runAtVoltage(double nVoltage) {
    return new RunCommand(
        () -> {
          io.runVoltage(nVoltage);
        },
        this);
  }

  public SequentialCommandGroup shootRight() {
    return shoot(Constants.shootingVoltage);
  }

  public SequentialCommandGroup shootLeft() {
    return shoot(-Constants.shootingVoltage);
  }

  SequentialCommandGroup shoot(double voltage) {
    return new SequentialCommandGroup(
        new InstantCommand(
            () -> {
              io.runVoltage(voltage);
            }),
        new WaitCommand(1),
        new InstantCommand(
            () -> {
              io.runVoltage(0);
            }));
  }

  public Command ortala() {
    return new FunctionalCommand(
        () -> {},
        () -> {
          if (inputs.sensor1 && !inputs.sensor2) {
            io.runVoltage(Constants.shooterAdjustingVoltage);
          } else if (inputs.sensor2 && !inputs.sensor1) {
            io.runVoltage(-Constants.shooterAdjustingVoltage);
          }
        },
        (Boolean supplied) -> {
          io.runVoltage(0);
        },
        () -> {
          if (inputs.sensor1 && inputs.sensor2) return true;
          return false;
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
