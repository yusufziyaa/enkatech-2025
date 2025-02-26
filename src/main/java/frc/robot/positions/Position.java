package frc.robot.positions;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public interface Position {
  public default Command getToZero() {
    return new InstantCommand(() -> {});
  }

  public default Command getToL2() {
    return new InstantCommand(() -> {});
  }

  public default Command getToL3() {
    return new InstantCommand(() -> {});
  }

  public default Command getToHangar() {
    return new InstantCommand(() -> {});
  }

  public default Command getToL4() {
    return new InstantCommand(() -> {});
  }

  public default Command getToGround() {
    return new InstantCommand(() -> {});
  }
}
