package frc.robot.positions;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.exterior_elevator.ExteriorElevator;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.interior_elevator.InteriorElevator;

public class GroundPosition implements Position {
    Intake intake;
    ExteriorElevator exterior;
    InteriorElevator interior;
    Arm arm;
    public GroundPosition(Intake intake,Arm arm,InteriorElevator interiorElevator,ExteriorElevator exteriorElevator) {
        this.intake = intake;
        this.exterior = exteriorElevator;
        this.interior = interiorElevator;
        this.arm = arm;
    }

    @Override
    public Command getToZero() {
        return new SequentialCommandGroup(
            new ParallelCommandGroup(arm.getToNull(),new WaitCommand(0.5)),
            interior.getToHigh(),
            arm.getToZero()
        );
    }

    @Override
    public Command getToHangar() {
        return new SequentialCommandGroup(
            new ParallelCommandGroup(arm.getToNull(),new WaitCommand(0.5)),
            interior.getToHigh(),
            arm.hangar()
        );
    }

    @Override
    public Command getToL2() {
        return new SequentialCommandGroup(
            new ParallelCommandGroup(arm.getToNull(),new WaitCommand(0.5)),
            interior.getToHigh(),
            arm.getToL2L3(),
            exterior.getToL2()
        );
    }

    @Override
    public Command getToL3() {
        return new SequentialCommandGroup(
            new ParallelCommandGroup(arm.getToNull(),new WaitCommand(0.5)),
            interior.getToHigh(),
            arm.getToL2L3(),
            exterior.getToL3()
        );
    }
}
