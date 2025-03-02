// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.tirmanma;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Tirmanma extends SubsystemBase {
  /** Creates a new Tirmanma. */
  TirmanmaIO io;

  public Tirmanma(TirmanmaIO io) {
    this.io = io;
  }

  public Command runVoltage(double v) {
    return new InstantCommand(
        () -> {
          io.runVoltage(v);
        });
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
