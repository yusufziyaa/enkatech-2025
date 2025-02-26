package frc.robot.subsystems.interior_elevator;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;

public class InteriorElevatorIOTalonFX implements InteriorElevatorIO {
  TalonFX m_motor;
  MotionMagicVoltage request = new MotionMagicVoltage(0);

  public InteriorElevatorIOTalonFX(int CANID) {
    m_motor = new TalonFX(CANID, "canivore");

    MotionMagicConfigs configs =
        new MotionMagicConfigs()
            .withMotionMagicAcceleration(150)
            .withMotionMagicCruiseVelocity(300);

    Slot0Configs cSlot0Configs =
        new Slot0Configs()
            .withGravityType(GravityTypeValue.Elevator_Static)
            .withKG(0.1)
            .withKP(10)
            .withKD(0);

    m_motor.getConfigurator().apply(configs);
    m_motor.getConfigurator().apply(cSlot0Configs);

    m_motor
        .getConfigurator()
        .apply(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));

    m_motor.setPosition(0);
  }

  @Override
  public double getPosition() {
    return m_motor.get();
  }

  @Override
  public void getToPosition(double position) {
    m_motor.setControl(request.withPosition(position));
  }

  @Override
  public void updateInputs(InteriorElevatorIOInputs inputs) {
    inputs.isAlive = m_motor.isAlive();
    inputs.isConnected = m_motor.isConnected();
    inputs.speed = m_motor.get();
    inputs.position = m_motor.getPosition().getValueAsDouble();
    inputs.appliedAmps = m_motor.getTorqueCurrent().getValueAsDouble();
    inputs.appliedVoltage = m_motor.getMotorVoltage().getValueAsDouble();
  }
}
