package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
  public Mechanism2d mech = null;

  public default void updateInputs(ElevatorInputs inputs) {}

  public default void adjustSetpoints(double angle, double elevatorLength, double intakeAngle) {}

  public default void runVelocity(double elevatorSpeed, double armSpeed, double intakeSpeed) {}

  public default double getElevatorExtEncoder() {
    return 0;
  }

  public default double getElevatorIntEncoder() {
    return 0;
  }

  public default double getArmEncoder() {
    return 0;
  }

  public default double getIntakeEncoder() {
    return 0;
  }

  public default void simulationPeriodic() {}

  @AutoLog
  public static class ElevatorInputs {
    double interiorElevatorState = 0;
    double exteriorElevatorState = 0;
    boolean interiorElevatorConnected = false;
    ;
    boolean exteriorElevatorConnected = false;
  }
}
