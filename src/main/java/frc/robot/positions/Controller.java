package frc.robot.positions;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.exterior_elevator.ExteriorElevator;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.interior_elevator.InteriorElevator;

public class Controller {

  ExteriorElevator exteriorElevator;
  InteriorElevator interiorElevator;
  Arm arm;
  Intake intake;

  Position currentPosition;

  Position groundPosition;
  Position zeroPosition;
  Position L2L3Position;
  Position HangarPosition;
  Position L4Position;

  public Controller(ExteriorElevator exterior, InteriorElevator interior, Arm arm, Intake intake) {
    this.exteriorElevator = exterior;
    this.interiorElevator = interior;
    this.arm = arm;
    this.intake = intake;

    currentPosition = new StartingPosition(intake, arm, interior, exterior);

    groundPosition = new GroundPosition(intake, arm, interiorElevator, exteriorElevator);
    zeroPosition = new ZeroPosition(intake, arm, interiorElevator, exteriorElevator);
    L2L3Position = new L2L3Position(intake, arm, interiorElevator, exteriorElevator);
    HangarPosition = new HangarPosition(intake, arm, interiorElevator, exteriorElevator);
    L4Position = new L4Position(intake, arm, interiorElevator, exteriorElevator);
  }

  public Command getToGround() {
    System.out.println(currentPosition.getClass().getCanonicalName() + " ground");
    Command toExecute = currentPosition.getToGround();
    currentPosition = groundPosition;
    return toExecute;
  }

  public Command getToZero() {
    System.out.println(currentPosition.getClass().getCanonicalName() + " zero");
    Command toExecute = currentPosition.getToZero();
    currentPosition = zeroPosition;
    return toExecute;
  }

  public Command getToL2() {
    System.out.println(currentPosition.getClass().getCanonicalName() + " l2");
    Command toExecute = currentPosition.getToL2();
    currentPosition = L2L3Position;
    return toExecute;
  }

  public Command getToL3() {
    System.out.println(currentPosition.getClass().getCanonicalName() + " l3");
    Command toExecute = currentPosition.getToL3();
    currentPosition = L2L3Position;
    return toExecute;
  }

  public Command getToL4() {
    System.out.println(currentPosition.getClass().getCanonicalName() + " l4");
    Command toExecute = currentPosition.getToL4();
    currentPosition = L4Position;
    return toExecute;
  }
}
