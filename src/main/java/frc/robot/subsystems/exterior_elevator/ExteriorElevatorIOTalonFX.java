package frc.robot.subsystems.exterior_elevator;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;

public class ExteriorElevatorIOTalonFX implements ExteriorElevatorIO {
  TalonFX m_motor;
  TalonFX m_slave;
  MotionMagicVoltage request = new MotionMagicVoltage(0);

  public ExteriorElevatorIOTalonFX(int CANID, int CANIDSlave) {
    m_motor = new TalonFX(CANID, "canivore");
    m_slave = new TalonFX(CANIDSlave, "canivore");

    m_slave.setControl(new Follower(CANID, true));

    m_motor.setPosition(0);

    MotionMagicConfigs configs =
        new MotionMagicConfigs()
            .withMotionMagicAcceleration(300)
            .withMotionMagicCruiseVelocity(200);

    Slot0Configs cSlot0Configs =
        new Slot0Configs()
            .withGravityType(GravityTypeValue.Elevator_Static)
            .withKG(0.2)
            .withKP(1)
            .withKD(0);

    m_motor.getConfigurator().apply(configs);
    m_motor.getConfigurator().apply(cSlot0Configs);

    m_slave.getConfigurator().apply(configs);
    m_slave.getConfigurator().apply(cSlot0Configs);
  }

  @Override
  public void getToPosition(double pos) {
    m_motor.setControl(request.withPosition(pos));
  }

  @Override
  public void updateInputs(ExteriorElevatorIOInputs inputs) {
    inputs.isAlive = m_motor.isAlive();
    inputs.isConnected = m_motor.isConnected();
    inputs.speed = m_motor.get();
    inputs.position = m_motor.getPosition().getValueAsDouble();
    inputs.appliedAmps = m_motor.getTorqueCurrent().getValueAsDouble();
    inputs.appliedVoltage = m_motor.getMotorVoltage().getValueAsDouble();

    inputs.slaveAppliedAmps = m_slave.getTorqueCurrent().getValueAsDouble();
    inputs.slaveAppliedVoltage = m_slave.getMotorVoltage().getValueAsDouble();
    inputs.slaveIsAlive = m_slave.isAlive();
    inputs.slaveIsConnected = m_slave.isConnected();
    inputs.slaveSpeed = m_slave.get();
  }
}
