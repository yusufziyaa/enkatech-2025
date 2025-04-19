// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.exterior_elevator.ExteriorElevator;
import frc.robot.subsystems.intake.Intake;
import frc.robot.util.LimelightHelpers;
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

  public Command shootIfAvab(Intake intake, ExteriorElevator exteriorElevator) {
    return new ConditionalCommand(
        shootInCorrectAngle(intake, exteriorElevator),
        new InstantCommand(() -> {}),
        () -> LimelightHelpers.getFiducialID("limelight") != -1);
  }

  public Command shootInCorrectAngle(Intake intake, ExteriorElevator exteriorElevator) {
    return new ConditionalCommand(
        new ConditionalCommand(
            shoot(Constants.shootingVoltage * -1),
            shoot(Constants.shootingVoltage),
            () -> intake.getDesired() == 1),
        new ConditionalCommand(
            shoot(Constants.shootingVoltage),
            shoot(-Constants.shootingVoltage),
            () -> intake.getDesired() == 1),
        () -> exteriorElevator.getState() == 3);
    /*return shoot(
    Constants.shootingVoltage
        * (intake.getDesired() == 0 ? 1 : -1)
        * (exteriorElevator.getState() == 3 ? -1 : 1));*/
  }

  public Command shootInCorrectAngle(
      Intake intake, ExteriorElevator exteriorElevator, double coef) {
    return new ConditionalCommand(
        new ConditionalCommand(
            shoot(Constants.shootingVoltage * -1 * coef),
            shoot(Constants.shootingVoltage * coef),
            () -> intake.getDesired() == 1),
        new ConditionalCommand(
            shoot(Constants.shootingVoltage * coef),
            shoot(-Constants.shootingVoltage * coef),
            () -> intake.getDesired() == 1),
        () -> exteriorElevator.getState() == 3);
    /*return shoot(
    Constants.shootingVoltage
        * (intake.getDesired() == 0 ? 1 : -1)
        * (exteriorElevator.getState() == 3 ? -1 : 1));*/
  }

  double direction = 1;

  public Command backup(Intake intake, ExteriorElevator exteriorElevator) {
    return new FunctionalCommand(
        () -> {
          direction =
              (intake.getDesired() == 1 ? 1 : -1) * (exteriorElevator.getState() == 3 ? -1 : 1);
          io.runVoltage(-Constants.shooterBackupVoltage * direction);
        },
        () -> {},
        (Boolean cons) -> {
          io.runVoltage(0);
        },
        () -> {
          if (!getSensor1() && direction == -1) return true;
          if (!getSensor2() && direction == 1) return true;
          return false;
        },
        this);
  }

  public SequentialCommandGroup shoot(double voltage) {
    return new SequentialCommandGroup(
        new InstantCommand(
            () -> {
              io.runVoltage(voltage);
            }),
        new WaitCommand(0.2),
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
          if (!getSensor1() && !getSensor2()) return true;
          return false;
        },
        this);
  }

  Boolean ciktimi = false;
  Integer init = -1;

  public Command betterOrtala() {
    return new FunctionalCommand(
        () -> {
          if (getSensor1()) init = -1;
          else init = 1;
        },
        () -> {
          if (getSensor1()) io.runVoltage(Constants.shooterAdjustingVoltage);
          if (!getSensor1() && getSensor2()) io.runVoltage(-Constants.shooterAdjustingVoltage);
        },
        (Boolean cons) -> {
          io.runVoltage(0);
        },
        () -> {
          if (init == -1 && !getSensor1()) return true;
          if (init == 1 && getSensor1()) return true;
          if (!getSensor1() && !getSensor2()) return true;
          return false;
        },
        this);
  }

  public Command az_ileri() {
    return new SequentialCommandGroup(
        runAtVoltage(-Constants.shooterBackupVoltage), new WaitCommand(0.2), runAtVoltage(0));
  }

  public Command ortala() {
    return new SequentialCommandGroup(
        new ParallelRaceGroup(
            new SequentialCommandGroup(
                ortala_raw(), runAtVoltage(-1), new WaitCommand(0.1), runAtVoltage(0)),
            new WaitCommand(1)),
        runAtVoltage(0));
  }

  public Command ortala_raw() {
    return new FunctionalCommand(
        () -> {
          ciktimi = false;
        },
        () -> {
          if (getSensor1() && !getSensor2()) {
            ciktimi = true;
            io.runVoltage(Constants.shooterAdjustingVoltage);
          } else if (getSensor2() && !getSensor1()) {
            ciktimi = true;
            io.runVoltage(-Constants.shooterAdjustingVoltage);
          } else if (!ciktimi) {
            io.runVoltage(Constants.shooterAdjustingVoltage);
          }
        },
        (Boolean supplied) -> {
          io.runVoltage(0);
        },
        () -> {
          if (getSensor1() && getSensor2() && ciktimi) return true;
          if (!getSensor1() && !getSensor2()) return true;
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
