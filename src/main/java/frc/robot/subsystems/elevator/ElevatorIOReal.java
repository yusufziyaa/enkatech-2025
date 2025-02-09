package frc.robot.subsystems.elevator;

import edu.wpi.first.math.controller.PIDController;

public class ElevatorIOReal implements ElevatorIO {
  MotorIO m_intake,m_arm,m_interior,m_exterior;

  public ElevatorIOReal(MotorIO intake, MotorIO arm, MotorIO interior, MotorIO exterior) {
    this.m_intake = intake;
    this.m_arm = arm;
    this.m_interior=interior;
    this.m_exterior=exterior;
  }

  @Override
  public void adjustSetpoints(double desiredArmAngle,double desiredElevatorLength,double desiredIntakeAngle) {
    m_intake.runTorque(
      desiredIntakeAngle
    );
    
    if (desiredElevatorLength > ElevatorConstants.innerElevatorMaximumSetpoint) {
      m_interior.runTorque(ElevatorConstants.innerElevatorMaximumSetpoint);
      m_exterior.runTorque(desiredElevatorLength - ElevatorConstants.innerElevatorMaximumSetpoint);
    } 
    
    m_arm.runTorque(
      desiredArmAngle
    );
  }
}
