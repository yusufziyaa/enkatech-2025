// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.exterior_elevator;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ExteriorElevator extends SubsystemBase {
  /** Creates a new ExteriorElevator. */
  ExteriorElevatorIO io_1, io_2;

  ExteriorElevatorIOInputsAutoLogged inputs1 = new ExteriorElevatorIOInputsAutoLogged();
  ExteriorElevatorIOInputsAutoLogged inputs2 = new ExteriorElevatorIOInputsAutoLogged();

  public ExteriorElevator(ExteriorElevatorIO io1, ExteriorElevatorIO io2) {
    this.io_1 = io1;
    this.io_2 = io2;
  }

  public void getToPosition(double pos) {
    io_1.getToPosition(pos);
    io_2.getToPosition(-pos);
  }

  @Override
  public void periodic() {
    io_1.updateInputs(inputs1);
    io_2.updateInputs(inputs2);
    // This method will be called once per scheduler run
  }
}
