// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
  /** Creates a new Elevator. */

  // FIXME: delete public
  public ElevatorIO io;

  ElevatorInputsAutoLogged inputs = new ElevatorInputsAutoLogged();

  public Elevator() {}

  public Elevator(ElevatorIO io) {
    this.io = io;
    // asansör
  }

  public void runVelocity(double elevatorSpeed, double armSpeed, double intakeSpeed) {
    io.runVelocity(elevatorSpeed, armSpeed, intakeSpeed);
  }

  public double getElevatorEncoder() {
    return io.getElevatorExtEncoder();
  }

  public double getArmEncoder() {
    return io.getArmEncoder();
  }

  public double getIntakeEncoder() {
    return io.getIntakeEncoder();
  }

  @Override
  public void simulationPeriodic() {
    io.simulationPeriodic();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);

    // io.adjustSetpoints(desiredArmAngle, desiredElevatorLength,
    // desiredIntakeAngle);

    // Logger.recordOutput("Mechanisms/IntakePosition", yesil_parca.getPosition);
    // This method will be called once per scheduler run
  }
}
