package frc.robot.subsystems.tirmanma;

import org.littletonrobotics.junction.AutoLog;

public interface TirmanmaIO {

  @AutoLog
  public static class TirmanmaIOInputs {
    public boolean isConnected = false;
    public double position = 0;
    public double speed = 0;
    public boolean isAlive = false;
    public double appliedVoltage = 0;
    public double appliedAmps = 0;
  }

  public default void runVoltage(double voltage) {}

  public default void updateInputs(TirmanmaIOInputs inputs) {}
}
