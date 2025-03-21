package frc.robot.subsystems.tirmanma;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

public class TirmanmaIOTalonFX implements TirmanmaIO {
  TalonFX m_motor;
  VoltageOut voltageOut = new VoltageOut(0);

  public TirmanmaIOTalonFX(int CANID) {
    m_motor = new TalonFX(CANID, "canivore");

    m_motor.setPosition(0);
  }

  public void updateInputs() {}

  @Override
  public void runVoltage(double voltage) {
    m_motor.setControl(voltageOut.withOutput(voltage));
  }

  @Override
  public void updateInputs(TirmanmaIOInputs inputs) {
    inputs.isAlive = m_motor.isAlive();
    inputs.isConnected = m_motor.isConnected();
    inputs.speed = m_motor.get();
    inputs.position = m_motor.getPosition().getValueAsDouble();
    inputs.appliedAmps = m_motor.getTorqueCurrent().getValueAsDouble();
    inputs.appliedVoltage = m_motor.getMotorVoltage().getValueAsDouble();
  }
}
