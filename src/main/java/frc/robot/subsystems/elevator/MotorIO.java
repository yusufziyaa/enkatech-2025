package frc.robot.subsystems.elevator;

public interface MotorIO {
  public default double getEncoderPosition() {
    return 0;
  }

  public default double getMaximumPosition() {
    return 0;
  }

  public default double getMinimumPosition() {
    return 0;
  }

  public default void setMaximumPosition(double pos) {}

  public default void setMinimumPosition(double pos) {}

  public default void runVelocity(double velocity) {}

  public default void runTorque(double torque) {} // requires ctre pro
}
