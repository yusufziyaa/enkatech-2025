package frc.robot.subsystems.interior_elevator;

import org.littletonrobotics.junction.AutoLog;

public interface InteriorElevatorIO {
  public default void getToPosition(double position) {}

  @AutoLog
  public static class InteriorElevatorIOInputs {
    public boolean isConnected = false;
    public double position = 0;
    public double speed = 0;
    public boolean isAlive = false;
    public double appliedVoltage = 0;
    public double appliedAmps = 0;
  }

  public default void updateInputs(InteriorElevatorIOInputs inputs) {}
}
