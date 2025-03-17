// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.gripper;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.shooter.Shooter;
import org.littletonrobotics.junction.Logger;

public class Gripper extends SubsystemBase {
  /** Creates a new Gripper. */
  GripperIO io;

  GripperIOInputsAutoLogged inputs = new GripperIOInputsAutoLogged();

  public Gripper() {}

  public Gripper(GripperIO io) {
    this.io = io;
  }

  public void setVoltage(double v) {
    io.runVoltage(v);
  }

  public Command runAtVoltage(double v) {
    return new InstantCommand(
        () -> {
          io.runVoltage(v);
        });
  }

  public Command gripPP(Shooter shooter) {
    return new FunctionalCommand(
        () -> {
          setVoltage(10);
        },
        () -> {},
        (Boolean cons) -> {
          setVoltage(0);
        },
        () -> {
          if (shooter.getSensor1() && shooter.getSensor2()) return true;
          return false;
        },
        this);
  }

  public Command gripTillSeen(Shooter shooter) {
    return new SequentialCommandGroup(
        new FunctionalCommand(
            () -> {},
            () -> {
              setVoltage(5);
            },
            (Boolean cons) -> {
              setVoltage(0);
            },
            () -> {
              if (shooter.getSensor1() && shooter.getSensor2()) return true;
              return false;
            },
            this),
        new WaitCommand(0.1),
        runAtVoltage(0));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(getName(), inputs);
    // This method will be called once per scheduler run
  }
}
