// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.interior_elevator;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class InteriorElevator extends SubsystemBase {
  /** Creates a new InteriorElevator. */
  InteriorElevatorIO io;

  InteriorElevatorIOInputsAutoLogged inputs = new InteriorElevatorIOInputsAutoLogged();

  public InteriorElevator(InteriorElevatorIO io) {
    this.io = io;
  }

  public void runPosition(double position) {
    io.runPosition(position);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(getName(), inputs);
    // This method will be called once per scheduler run
  }
}
