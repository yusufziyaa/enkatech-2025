// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.exterior_elevator.ExteriorElevator;
import frc.robot.subsystems.intake.Intake;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  ShooterIO io;
  ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

  public Shooter() {}

  public Shooter(ShooterIO io) {
    this.io = io;
  }

  public boolean getSensor1() {
    return !inputs.sensor1;
  }

  public boolean getSensor2() {
    return !inputs.sensor2;
  }

  public Command runAtVoltage(double nVoltage) {
    return new InstantCommand(
        () -> {
          io.runVoltage(nVoltage);
        },
        this);
  }

  public void runVoltage(double v) {
    io.runVoltage(v);
  }

  public SequentialCommandGroup shootRight() {
    return shoot(2);
  }

  public SequentialCommandGroup shootLeft() {
    return shoot(-2);
  }

  public Command shootInCorrectAngle(Intake intake, ExteriorElevator exteriorElevator) {
    return new ConditionalCommand(
        new ConditionalCommand(
            shoot(Constants.shootingVoltage * -1),
            shoot(Constants.shootingVoltage),
            () -> intake.getDesired() == 0),
        new ConditionalCommand(
            shoot(Constants.shootingVoltage),
            shoot(-Constants.shootingVoltage),
            () -> intake.getDesired() == 0),
        () -> exteriorElevator.getState() == 3);
    /*return shoot(
    Constants.shootingVoltage
        * (intake.getDesired() == 0 ? 1 : -1)
        * (exteriorElevator.getState() == 3 ? -1 : 1));*/
  }

  public Command backup(Intake intake, ExteriorElevator exteriorElevator) {
    return new SequentialCommandGroup(
        new InstantCommand(
            () -> {
              io.runVoltage(
                  -Constants.shooterBackupVoltage
                      * (intake.getDesired() == 0 ? 1 : -1)
                      * (exteriorElevator.getState() == 3 ? -1 : 1));
            }),
        new WaitCommand(0.2),
        new InstantCommand(
            () -> {
              io.runVoltage(0);
            }));
  }

  public SequentialCommandGroup shoot(double voltage) {
    return new SequentialCommandGroup(
        new InstantCommand(
            () -> {
              io.runVoltage(voltage);
            }),
        new WaitCommand(0.3),
        new InstantCommand(
            () -> {
              io.runVoltage(0);
            }));
  }

  public Command waitToOrtala() {
    return new FunctionalCommand(
        () -> {},
        () -> {
          if (getSensor1() && !getSensor2()) io.runVoltage(-Constants.shooterAdjustingVoltage);
          else if (!getSensor1() && getSensor2()) io.runVoltage(Constants.shooterAdjustingVoltage);
        },
        (Boolean cons) -> {
          io.runVoltage(0);
        },
        () -> {
          if (getSensor1() && getSensor2()) return true;
          return false;
        },
        this);
  }

  Boolean ciktimi = false;

  public Command ortala() {
    return new FunctionalCommand(
        () -> {
          ciktimi = false;
        },
        () -> {
          if (getSensor1() && !getSensor2()) {
            ciktimi = true;
            io.runVoltage(-Constants.shooterAdjustingVoltage);
          } else if (getSensor2() && !getSensor1()) {
            ciktimi = true;
            io.runVoltage(Constants.shooterAdjustingVoltage);
          } else if (!ciktimi) {
            io.runVoltage(Constants.shooterAdjustingVoltage);
          }
        },
        (Boolean supplied) -> {
          io.runVoltage(0);
        },
        () -> {
          if (getSensor1() && getSensor2() && ciktimi) return true;
          if (!getSensor1() && !getSensor2() && ciktimi) return true;
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
