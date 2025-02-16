package frc.robot.subsystems.gripper;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

public class GripperIOTalonFX implements GripperIO {
  TalonFX m_motor;
  VoltageOut voltageOut = new VoltageOut(0);

  public GripperIOTalonFX(int gripperCANID) {
    m_motor = new TalonFX(gripperCANID, "canivore");
  }

  @Override
  public void runVoltage(double voltage) {
    m_motor.setControl(voltageOut.withOutput(voltage));
  }

  @Override
  public void hold() {
    m_motor.setControl(voltageOut.withOutput(0));
  }

  @Override
  public void updateInputs(GripperIOInputs inputs) {
    inputs.isAlive = m_motor.isAlive();
    inputs.isConnected = m_motor.isConnected();
    inputs.speed = m_motor.get();
    inputs.position = m_motor.getPosition().getValueAsDouble();
    inputs.appliedAmps = m_motor.getTorqueCurrent().getValueAsDouble();
    inputs.appliedVoltage = m_motor.getMotorVoltage().getValueAsDouble();
  }
}
