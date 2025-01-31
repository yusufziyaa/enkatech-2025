package frc.robot.subsystems.elevator.exterior;

import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.Constants;
import frc.robot.subsystems.elevator.MotorIO;

public class ExteriorIOTalonFX implements MotorIO {
  private final TalonFX m_motor;
  private final CANcoder cancoder;

  private final TorqueCurrentFOC torque_req = new TorqueCurrentFOC(0);

  public ExteriorIOTalonFX(int motor_id, int cancoder_id) {
    m_motor = new TalonFX(motor_id, "canivore");
    cancoder = new CANcoder(cancoder_id, "canivore");

    cancoder.getConfigurator().apply(Constants.initialCancoderConfig);
    // handle offsets
    // TODO
  }

  @Override
  public double getEncoderPosition() {
    return cancoder.getPosition().getValueAsDouble();
  }

  @Override
  public double getMaximumPosition() {
    return 0;
  }

  @Override
  public double getMinimumPosition() {
    return 0;
  }

  @Override
  public void setMaximumPosition(double pos) {}

  @Override
  public void setMinimumPosition(double pos) {}

  @Override
  public void runVelocity(double velocity) {}

  @Override
  public void runTorque(double torque) {
    m_motor.setControl(torque_req.withOutput(torque).withMaxAbsDutyCycle(0.5));
  } // requires ctre pro
}
