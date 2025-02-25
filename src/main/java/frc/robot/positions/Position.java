package frc.robot.positions;

import edu.wpi.first.wpilibj2.command.Command;

public interface Position {
  public default Command getToZero() {
    return null;
  }

  public default Command getToL2() {
    return null;
  }

  public default Command getToL3() {
    return null;
  }

  public default Command getToHangar() {
    return null;
  }

  public default Command getToL4() {
    return null;
  }

  public default Command getToGround() {
    return null;
  }
}
