package frc.robot.subsystems.exterior_elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ExteriorElevatorIO {
  public default void getToPosition(double pos) {}

  @AutoLog
  public static class ExteriorElevatorIOInputs {
    public boolean isConnected = false;
    public double position = 0;
    public double speed = 0;
    public boolean isAlive = false;
    public double appliedVoltage = 0;
    public double appliedAmps = 0;
  }

  public default void updateInputs(ExteriorElevatorIOInputs inputs) {}
}
