// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.AutoCommands;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.DriverScoreCommands;
import frc.robot.commands.Otonom;
import frc.robot.generated.TunerConstants;
import frc.robot.positions.Controller;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmIOSim;
import frc.robot.subsystems.arm.ArmIOTalonFX;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.GyroIOSim;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.exterior_elevator.ExteriorElevator;
import frc.robot.subsystems.exterior_elevator.ExteriorElevatorIOSim;
import frc.robot.subsystems.exterior_elevator.ExteriorElevatorIOTalonFX;
import frc.robot.subsystems.gripper.Gripper;
import frc.robot.subsystems.gripper.GripperIOSim;
import frc.robot.subsystems.gripper.GripperIOTalonFX;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.intake.IntakeIOTalonFX;
import frc.robot.subsystems.interior_elevator.InteriorElevator;
import frc.robot.subsystems.interior_elevator.InteriorElevatorIOSim;
import frc.robot.subsystems.interior_elevator.InteriorElevatorIOTalonFX;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.subsystems.shooter.ShooterIOTalonFX;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhoton;
import frc.robot.subsystems.vision.VisionIOSim;
import frc.robot.util.General;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;

  // Controller
  // private final CommandXboxController controller = new
  // CommandXboxController(0);
  private final CommandXboxController controller = new CommandXboxController(0);
  private final CommandXboxController operator = new CommandXboxController(1);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;
  SwerveDriveSimulation sim;

  private final Vision vision;
  private Shooter shooter;
  private Gripper gripper;
  private Arm arm;
  private InteriorElevator interiorElevator;
  private ExteriorElevator exteriorElevator;
  private Intake intake;

  private Controller positionController;

  public RobotContainer() {

    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));

        // TODO:implement real io
        vision = new Vision(new VisionIOPhoton("camera1", "camera2", "limelight"));

        shooter = new Shooter(new ShooterIOTalonFX(Constants.ShooterCANID));
        gripper = new Gripper(new GripperIOTalonFX(Constants.GripperCANID));

        arm = new Arm(new ArmIOTalonFX(Constants.ArmCANID));

        intake = new Intake(new IntakeIOTalonFX(Constants.IntakeCANID));

        interiorElevator =
            new InteriorElevator(new InteriorElevatorIOTalonFX(Constants.InteriorElevatorCANID));
        exteriorElevator =
            new ExteriorElevator(
                new ExteriorElevatorIOTalonFX(
                    Constants.ExteriorElevatorCANID_AKU, Constants.ExteriorElevatorCANID_N));
        break;

      case SIM:

        // Sim robot, instantiate physics sim IO implementations
        this.sim = new SwerveDriveSimulation(Drive.config, Constants.initialPose);
        drive =
            new Drive(
                new GyroIOSim(sim.getGyroSimulation()),
                new ModuleIOSim(sim.getModules()[0]),
                new ModuleIOSim(sim.getModules()[1]),
                new ModuleIOSim(sim.getModules()[2]),
                new ModuleIOSim(sim.getModules()[3]));

        vision = new Vision(new VisionIOSim());

        shooter = new Shooter(new ShooterIOSim());
        gripper = new Gripper(new GripperIOSim());
        arm = new Arm(new ArmIOSim());
        intake = new Intake(new IntakeIOSim());
        interiorElevator = new InteriorElevator(new InteriorElevatorIOSim());
        exteriorElevator = new ExteriorElevator(new ExteriorElevatorIOSim());

        SimulatedArena.getInstance().addDriveTrainSimulation(sim);

        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        vision = new Vision(new VisionIO() {});

        break;
    }
    drive.setPose(Constants.initialPose);

    positionController = new Controller(exteriorElevator, interiorElevator, arm, intake);

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
    // Configure the button bindings
    configureButtonBindings();
  }

  private void configureButtonBindings() {
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX()));
    controller.a().onTrue(new Otonom(interiorElevator, exteriorElevator, intake, arm));
    controller.b().onTrue(DriverScoreCommands.zeroToL3(intake, arm, exteriorElevator));
    controller.povUp().onTrue(interiorElevator.getToHigh());
    controller.povDown().onTrue(interiorElevator.getToLow());
    controller
        .x()
        .onTrue(DriverScoreCommands.retreatL4(exteriorElevator, interiorElevator, arm, intake));

    controller
        .y()
        .onTrue(DriverScoreCommands.Hangar(interiorElevator, exteriorElevator, intake, arm));

    controller.leftBumper().onTrue(DriverScoreCommands.zeroToL2(intake, arm, exteriorElevator));

    controller.rightBumper().onTrue(gripper.runAtVoltage(9)).onFalse(gripper.runAtVoltage(0));

    controller.povRight().onTrue(shooter.shootRight());
    controller.povLeft().onTrue(shooter.shootLeft());

    controller.button(7).onTrue(intake.adjustToCenter());
    controller
        .button(8)
        .onTrue(
            AutoCommands.getPathfindingCommand(drive, new Pose2d(15, 4, new Rotation2d()), false));
  }

  public Command getAutonomousCommand() {
    // return new ScoreCoralCommand(drive, vision);
    // return ElevatorCommands.adjustTo(elevator, relativePose);
    // FIXME: when the autonomous command ends, robot sometimes keeps going at a random velocity
    // continiously
    return AutoCommands.getPathfindingCommand(drive, new Pose2d(6, 6, new Rotation2d()), false);
    // return new AutoCycle(drive, vision, elevator);
  }

  public void updateCamera() {
    vision.simulationPeriodic(sim.getSimulatedDriveTrainPose());
  }

  public void displaySim() { // sadece simülasyon esnasında çalışmalı
    // advantage scope için robot konumunu publishleme
    Logger.recordOutput("FieldSimulation/RobotPosition", sim.getSimulatedDriveTrainPose());

    // coral konumlarını publishleme
    Pose3d[] corals = SimulatedArena.getInstance().getGamePiecesArrayByType("Coral");
    Logger.recordOutput("FieldSimulation/CoralPos", corals);

    Logger.recordOutput(
        "FieldSimulation/OdometryError",
        General.DistancePose2d(sim.getSimulatedDriveTrainPose(), drive.getPose()));

    // Logger.recordOutput("RelativePosition", new Pose2d(drive.getPose() + relativePose);
  }

  public void periodic() {
    // Logger.recordOutput()"id", vision.getBestTargetL().fiducialId);
    /*Optional<EstimatedRobotPose> robotPoseL = vision.getPoseL();
    if (robotPoseL.isPresent()) {
      drive.addVisionMeasurement(
          robotPoseL.get().estimatedPose.toPose2d(),
          robotPoseL.get().timestampSeconds,
          vision.getEstimationStdDevs(0));
    }

    Optional<EstimatedRobotPose> robotPoseR = vision.getPoseR();
    if (robotPoseR.isPresent()) {
      drive.addVisionMeasurement(
          robotPoseR.get().estimatedPose.toPose2d(),
          robotPoseR.get().timestampSeconds,
          vision.getEstimationStdDevs(1));
    }*/
  }
}
