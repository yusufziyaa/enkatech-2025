// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.interior_elevator;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class InteriorElevator extends SubsystemBase {
  /** Creates a new InteriorElevator. */
  InteriorElevatorIO io;

  InteriorElevatorIOInputsAutoLogged inputs = new InteriorElevatorIOInputsAutoLogged();

  public InteriorElevator(InteriorElevatorIO io) {
    this.io = io;
  }

  public double getPosition() {
    return io.getPosition();
  }

  public Command getToHigh() {
    return new FunctionalCommand(
        () -> {
          io.getToPosition(115);
        },
        () -> {},
        (Boolean cons) -> {},
        () -> {
          return inputs.position > 111;
        },
        this);
  }

  public Command getToLow() {
    return new InstantCommand(
        () -> {
          io.getToPosition(0); // essentially 0
        });
  }

  public Command waitTillLow() {
    return new FunctionalCommand(
        () -> {
          io.getToPosition(0);
        },
        () -> {},
        (Boolean cons) -> {},
        () -> {
          return inputs.position < 3;
        },
        this);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(getName(), inputs);

    SmartDashboard.putNumber("InteriorPosition", inputs.position);
    // This method will be called once per scheduler run
  }
}
