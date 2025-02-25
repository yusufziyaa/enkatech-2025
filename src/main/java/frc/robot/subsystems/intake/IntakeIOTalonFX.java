package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

public class IntakeIOTalonFX implements IntakeIO {
  TalonFX m_motor;

  MotionMagicVoltage request = new MotionMagicVoltage(0);

  @Override
  public void runToPosition(double pos) {
    m_motor.setControl(request.withPosition(pos));
  }

  public IntakeIOTalonFX(int CANID) {
    m_motor = new TalonFX(CANID, "canivore");

    MotionMagicConfigs configs =
        new MotionMagicConfigs().withMotionMagicAcceleration(10).withMotionMagicCruiseVelocity(20);

    Slot0Configs cSlot0Configs = new Slot0Configs().withKP(5).withKD(0);

    m_motor.getConfigurator().apply(configs);
    m_motor.getConfigurator().apply(cSlot0Configs);
    m_motor.setPosition(0);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.isAlive = m_motor.isAlive();
    inputs.isConnected = m_motor.isConnected();
    inputs.speed = m_motor.get();
    inputs.position = m_motor.getPosition().getValueAsDouble();
    inputs.appliedAmps = m_motor.getTorqueCurrent().getValueAsDouble();
    inputs.appliedVoltage = m_motor.getMotorVoltage().getValueAsDouble();
  }
}
