// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Arm extends SubsystemBase {
  /** Creates a new Arm. */
  ArmIO io;

  ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();
  PIDController controller = new PIDController(0.5, 0, 0);

  public Arm() {}

  public Arm(ArmIO io) {
    this.io = io;
  }

  public Command hangar() {
    return new InstantCommand(
        () -> {
          io.getToAngle(0.2);
        });
  }

  public Command asansorHareket() {
    return new FunctionalCommand(
        () -> {
          io.getToAngle(0.2);
        },
        () -> {},
        (Boolean cons) -> {},
        () -> {
          return Math.abs(inputs.position - 0.2) < 0.03;
        },
        this);
  }

  public Command hangar2() {
    return new FunctionalCommand(
        () -> {
          io.getToAngle(-0.025);
        },
        () -> {},
        (Boolean cons) -> {},
        () -> {
          return Math.abs(inputs.position + 0.025) < 0.03;
        },
        this);
  }

  public Command getToL4() {
    return new InstantCommand(
        () -> {
          io.getToAngle(0.33);
        },
        this);
  }

  public Command getToZero() {
    return new FunctionalCommand(
        () -> {
          io.getToAngle(-0.15);
        },
        () -> {},
        (Boolean cons) -> {
          io.runVoltage(0);
        },
        () -> {
          return inputs.position < -0.03;
        },
        this);
  }

  public Command getToNull() {
    return new InstantCommand(
        () -> {
          io.getToAngle(0);
        });
  }

  public Command waitTillNull() {
    return new FunctionalCommand(
        () -> {
          io.getToAngle(0);
        },
        () -> {},
        (Boolean cons) -> {},
        () -> {
          return inputs.position < 0.1;
        },
        this);
  }

  public Command runVoltageZero() {
    return new InstantCommand(
        () -> {
          io.runVoltage(0);
        });
  }

  public Command getToL2L3() {
    return new FunctionalCommand(
        () -> {
          io.getToAngle(0.63);
        },
        () -> {},
        (Boolean cons) -> {
          io.runVoltage(0);
        },
        () -> {
          return inputs.position > 0.50;
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
