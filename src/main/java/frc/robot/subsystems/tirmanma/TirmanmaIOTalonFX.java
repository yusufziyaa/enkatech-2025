package frc.robot.subsystems.tirmanma;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

public class TirmanmaIOTalonFX implements TirmanmaIO {
  TalonFX m_motor;
  VoltageOut request = new VoltageOut(0);

  public TirmanmaIOTalonFX(int CANID) {
    m_motor = new TalonFX(CANID);
  }

  @Override
  public void runVoltage(double v) {
    m_motor.setControl(request.withOutput(v));
  }
}
