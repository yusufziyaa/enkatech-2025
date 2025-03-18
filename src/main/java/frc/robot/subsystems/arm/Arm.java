// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.exterior_elevator.ExteriorElevator;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.interior_elevator.InteriorElevator;
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
          io.getToAngle(0.15);
        });
  }

  public Command instantHangar2() {
    return new InstantCommand(
        () -> {
          io.getToAngle(-0.025);
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

  public Command start() {
    return new InstantCommand(
        () -> {
          io.getToAngle(0.23);
        });
  }

  public Command hangar2() {
    return new FunctionalCommand(
        () -> {
          io.getToAngle(-0.027);
        },
        () -> {},
        (Boolean cons) -> {},
        () -> {
          return Math.abs(inputs.position + 0.025) < 0.03;
        },
        this);
  }

  public Command retreatCommand(
      InteriorElevator interior, Intake intake, ExteriorElevator exterior) {

    // new SequentialCommandGroup()

    SmartDashboard.putNumber("armpos2", io.getPosition());
    SmartDashboard.putNumber("asansor2", interior.getPosition());
    if (io.getPosition() > 0.2 && interior.getPosition() < 5) {
      SmartDashboard.putNumber("command", 1);
      return new SequentialCommandGroup(
          exterior.getToGround(),
          asansorHareket(),
          intake.adjustToCenter(),
          interior.getToHigh(),
          getToZero());
    } else if (interior.getPosition() < 5) {
      SmartDashboard.putNumber("command", 2);

      return new SequentialCommandGroup(
          exterior.getToGround(), intake.adjustToCenter(), interior.getToHigh(), getToZero());
    } else {
      SmartDashboard.putNumber("command", 3);
      return new SequentialCommandGroup(
          exterior.getToGround(), intake.adjustToCenter(), getToZero());
    }
  }

  public Command getToL4() {
    return new InstantCommand(
        () -> {
          // double getto = SmartDashboard.getNumber("ArmL4Setpoint", 0.335);
          // System.out.println(getto);
          double getto = 0.295;
          io.getToAngle(getto);
        },
        this);
  }

  public Command instantZero() {
    return new InstantCommand(
        () -> {
          io.getToAngle(-0.15);
        });
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
          return inputs.position < 0;
        },
        this);
  }

  public Command getToNull() {
    return new FunctionalCommand(
        () -> {
          io.getToAngle(-0.07);
        },
        () -> {},
        (Boolean cons) -> {
          io.runVoltage(0);
        },
        () -> {
          return Math.abs(inputs.position + 0.07) < 0.03;
        },
        this);
  }

  // TOP ÇIKARMA: 0.52 arm, 0.42 dist, -5V grip
  // Tırmanma butona basıldıkça istediğimiz konuma ilerleyecek

  public Command waitTillNull() {
    return new FunctionalCommand(
        () -> {
          io.getToAngle(0);
        },
        () -> {},
        (Boolean cons) -> {},
        () -> {
          return Math.abs(inputs.position) < 0.1;
        },
        this);
  }

  public Command runVoltageZero() {
    return new InstantCommand(
        () -> {
          io.runVoltage(0);
        });
  }

  public Command getToTop() {
    return new FunctionalCommand(
        () -> {
          io.getToAngle(0.57);
        },
        () -> {},
        (Boolean cons) -> {},
        () -> {
          return Math.abs(inputs.position - 0.57) < 0.1;
        },
        this);
  }

  public Command getToL2L3() {
    return new FunctionalCommand(
        () -> {
          io.getToAngle(0.7);
        },
        () -> {},
        (Boolean cons) -> {
          io.runVoltage(0);
        },
        () -> {
          return inputs.position > 0.6;
        },
        this);
  }

  public Command NL4() {
    return new FunctionalCommand(
        () -> {
          io.getToAngle(0.27);
        },
        () -> {},
        (Boolean cons) -> {},
        () -> {
          return Math.abs(inputs.position - 0.27) < 0.1;
        },
        this);
  }

  public Command NHangar() {
    return new FunctionalCommand(
        () -> {
          io.getToAngle(0.02);
        },
        () -> {},
        (Boolean cons) -> {},
        () -> true,
        this);
  }

  public Command NL2L3() {
    return new FunctionalCommand(
        () -> {
          io.getToAngle(0.6);
        },
        () -> {},
        (Boolean cons) -> {
          io.runVoltage(0);
        },
        () -> {
          return Math.abs(inputs.position - 0.6) < 0.03;
        },
        this);
  }

  public Command NTop() {
    return new FunctionalCommand(
        () -> {
          io.getToAngle(0.52);
        },
        () -> {},
        (Boolean cons) -> {},
        () -> {
          return Math.abs(inputs.position - 0.52) < 0.03;
        },
        this);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(getName(), inputs);

    SmartDashboard.putNumber("armpos", io.getPosition());
    // This method will be called once per scheduler run
  }
}
