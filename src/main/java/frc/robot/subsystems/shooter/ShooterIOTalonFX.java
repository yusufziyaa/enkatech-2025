package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

public class ShooterIOTalonFX implements ShooterIO {
  TalonFX m_motor;
  VoltageOut voltageOut = new VoltageOut(0);

  public ShooterIOTalonFX(int canid) {
    m_motor = new TalonFX(canid, "canivore");
  }

  @Override
  public void runVoltage(double voltage) {
    m_motor.setControl(voltageOut.withOutput(voltage));
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    inputs.isAlive = m_motor.isAlive();
    inputs.isConnected = m_motor.isConnected();
    inputs.speed = m_motor.get();
    inputs.position = m_motor.getPosition().getValueAsDouble();
    inputs.appliedAmps = m_motor.getTorqueCurrent().getValueAsDouble();
    inputs.appliedVoltage = m_motor.getMotorVoltage().getValueAsDouble();
  }
}
