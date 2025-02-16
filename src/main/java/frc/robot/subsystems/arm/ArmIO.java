package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
  public default void getToAngle(double angle) {}

  @AutoLog
  public static class ArmIOInputs {
    public boolean isConnected = false;
    public double position = 0;
    public double speed = 0;
    public boolean isAlive = false;
    public double appliedVoltage = 0;
    public double appliedAmps = 0;
  }

  public default void updateInputs(ArmIOInputs inputs) {}
}
