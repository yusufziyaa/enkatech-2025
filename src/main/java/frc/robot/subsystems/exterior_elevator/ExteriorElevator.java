// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.exterior_elevator;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ExteriorElevator extends SubsystemBase {
  /** Creates a new ExteriorElevator. */
  ExteriorElevatorIO io;

  ExteriorElevatorIOInputsAutoLogged inputs = new ExteriorElevatorIOInputsAutoLogged();

  public ExteriorElevator(ExteriorElevatorIO io) {
    this.io = io;
  }

  public void getToPosition(double pos) {
    io.getToPosition(pos);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    // This method will be called once per scheduler run
  }
}
