package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.Constants;

public class MotorIOTalonFX implements MotorIO {
  private final TalonFX m_motor;
  private final CANcoder cancoder;

  private final TorqueCurrentFOC torqueCurrentFOC = new TorqueCurrentFOC(0);
  private final VelocityDutyCycle velocityDutyCycle = new VelocityDutyCycle(0);

  private final MotionMagicTorqueCurrentFOC mmTorqueControl = new MotionMagicTorqueCurrentFOC(0);

  public MotorIOTalonFX(int motor_id, int cancoder_id, Slot0Configs config) {
    m_motor = new TalonFX(motor_id, "canivore");
    cancoder = new CANcoder(cancoder_id, "canivore");

    cancoder.getConfigurator().apply(Constants.initialCancoderConfig);
    m_motor.getConfigurator().apply(config);
    // handle offsets
    // TODO
  }

  @Override
  public double getEncoderPosition() {
    return cancoder.getPosition().getValueAsDouble();
  }

  @Override
  public void runVelocity(double velocity) {
    m_motor.setControl(velocityDutyCycle.withVelocity(velocity));
  }

  @Override
  public void runTorque(double position) {
    m_motor.setControl(mmTorqueControl.withPosition(position));
  } // requires ctre pro
}
