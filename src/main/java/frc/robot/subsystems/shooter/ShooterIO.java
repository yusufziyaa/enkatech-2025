package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
  public default void runVoltage(double voltage) {}

  @AutoLog
  public static class ShooterIOInputs {
    public boolean isConnected = false;
    public double position = 0;
    public double speed = 0;
    public boolean isAlive = false;
    public double appliedVoltage = 0;
    public double appliedAmps = 0;

    public boolean sensor1 = false;
    public boolean sensor2 = false;
  }

  public default void updateInputs(ShooterIOInputs inputs) {}
}
