package frc.robot.subsystems.arm;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;

public class ArmIOTalonFX implements ArmIO {
  TalonFX m_motor;
  MotionMagicTorqueCurrentFOC request = new MotionMagicTorqueCurrentFOC(0);

  public ArmIOTalonFX(int armCANID) {
    m_motor = new TalonFX(armCANID, "canivore");
    m_motor
        .getConfigurator()
        .apply(
            new MotionMagicConfigs()
                .withMotionMagicAcceleration(2)
                .withMotionMagicCruiseVelocity(1));
    m_motor.setPosition(0.1841408);
    // TODO: tune, use setPosition to overcome Arm_cosine control type. probably no need for ratio
    // tuning because there is no reductory (or maybe there  is??)
    m_motor
        .getConfigurator()
        .apply(
            new Slot0Configs()
                .withGravityType(GravityTypeValue.Arm_Cosine)
                .withKG(0.765)
                .withKP(30)
                .withKD(0.1));
    m_motor.getConfigurator().apply(new FeedbackConfigs().withSensorToMechanismRatio(25));
  }

  @Override
  public void getToAngle(double angle) {
    m_motor.setControl(request.withPosition(angle));
  }

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    inputs.isAlive = m_motor.isAlive();
    inputs.isConnected = m_motor.isConnected();
    inputs.speed = m_motor.get();
    inputs.position = m_motor.getPosition().getValueAsDouble();
    inputs.appliedAmps = m_motor.getTorqueCurrent().getValueAsDouble();
    inputs.appliedVoltage = m_motor.getMotorVoltage().getValueAsDouble();
  }
}
