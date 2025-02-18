package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {

  public default void runToPosition(double pos) {}

  @AutoLog
  public static class IntakeIOInputs {
    public boolean isConnected = false;
    public double position = 0;
    public double speed = 0;
    public boolean isAlive = false;
    public double appliedVoltage = 0;
    public double appliedAmps = 0;
  }

  public default void updateInputs(IntakeIOInputs inputs) {}
}
