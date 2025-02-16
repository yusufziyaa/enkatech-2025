package frc.robot.subsystems.gripper;

import org.littletonrobotics.junction.AutoLog;

public interface GripperIO {
  public default void runVoltage(double voltage) {}

  public default void hold() {}

  @AutoLog
  public static class GripperIOInputs {
    public boolean isConnected = false;
    public double position = 0;
    public double speed = 0;
    public boolean isAlive = false;
    public double appliedVoltage = 0;
    public double appliedAmps = 0;
  }

  public default void updateInputs(GripperIOInputs inputs) {}
}
