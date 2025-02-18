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

    public double slaveAppliedVoltage = 0;
    public double slaveAppliedAmps = 0;

    public boolean slaveIsConnected = false;
    public double slaveSpeed = 0;
    public boolean slaveIsAlive = false;
  }

  public default void updateInputs(ExteriorElevatorIOInputs inputs) {}
}
