package frc.robot.positions;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.exterior_elevator.ExteriorElevator;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.interior_elevator.InteriorElevator;

public class L2L3Position implements Position {
        Intake intake;
    ExteriorElevator exterior;
    InteriorElevator interior;
    Arm arm;
    public L2L3Position(Intake intake,Arm arm,InteriorElevator interiorElevator,ExteriorElevator exteriorElevator) {
        this.intake = intake;
        this.exterior = exteriorElevator;
        this.interior = interiorElevator;
        this.arm = arm;
    }
    @Override
    public Command getToZero() {
        return new SequentialCommandGroup(
            exterior.getToGround(),
            arm.getToZero()
        );  
    }

    @Override
    public Command getToL2() {
        return new SequentialCommandGroup(
            exterior.getToL2()
        );  
    }

    @Override
    public Command getToL3() {
        return new SequentialCommandGroup(
            exterior.getToL3()
        );
    }

    @Override
    public Command getToHangar() {
        return new SequentialCommandGroup(
            exterior.getToGround(),
            arm.hangar()
        );
    }

    @Override
    public Command getToL4() {
        return new SequentialCommandGroup(
            exterior.getToL4(),
            arm.getToL4()
        );
    }

    @Override
    public Command getToGround() {
        return new SequentialCommandGroup(
            
        );
    }
}
