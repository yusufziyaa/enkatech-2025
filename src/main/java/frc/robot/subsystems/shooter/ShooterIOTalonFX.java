package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;

public class ShooterIOTalonFX implements ShooterIO {
  TalonFX m_motor;
  VoltageOut voltageOut = new VoltageOut(0);

  DigitalInput distance1 = new DigitalInput(Constants.MZ80_1_ID);
  DigitalInput distance2 = new DigitalInput(Constants.MZ80_2_ID);

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

    inputs.sensor1 =
        !distance1.get(); // eğer cisim varsa true, yoksa false olacak şekilde ters çevirilmiş
    inputs.sensor2 = !distance2.get();
  }
}
