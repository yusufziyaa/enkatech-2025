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
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.ScoreCoralCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.GyroIOSim;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIOReal;
import frc.robot.subsystems.elevator.arm.ArmIOTalonFX;
import frc.robot.subsystems.elevator.exterior.ExteriorIOTalonFX;
import frc.robot.subsystems.elevator.intake.IntakeIOTalonFX;
import frc.robot.subsystems.elevator.interior.InteriorIOTalonFX;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIOPhoton;
import frc.robot.subsystems.vision.VisionIOSim;
import java.util.Optional;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoral;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.photonvision.EstimatedRobotPose;

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
  private final Joystick joystick = new Joystick(0);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;
  SwerveDriveSimulation sim;

  // SIMULASYON BASLANGIC ROBOT KOORDINATI
  Pose2d initialPos = Constants.initialPose;

  private final Vision vision;
  private final Elevator elevator;

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
        vision = new Vision(new VisionIOPhoton());

        elevator = new Elevator();

        break;

      case SIM:
        elevator =
            new Elevator(
                new ElevatorIOReal(
                    new IntakeIOTalonFX(0, 1),
                    new ArmIOTalonFX(1, 2),
                    new InteriorIOTalonFX(3, 4),
                    new ExteriorIOTalonFX(5, 6)));

        // Sim robot, instantiate physics sim IO implementations
        this.sim = new SwerveDriveSimulation(Drive.config, initialPos);
        drive =
            new Drive(
                new GyroIOSim(sim.getGyroSimulation()),
                new ModuleIOSim(sim.getModules()[0]),
                new ModuleIOSim(sim.getModules()[1]),
                new ModuleIOSim(sim.getModules()[2]),
                new ModuleIOSim(sim.getModules()[3]));

        vision = new Vision(new VisionIOSim());

        SimulatedArena.getInstance()
            .addGamePiece(new ReefscapeCoral(new Pose2d(0, 0, new Rotation2d())));

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
        vision = new Vision();
        elevator = new Elevator();
        break;
    }
    drive.setPose(initialPos);

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Configure the button bindings
    configureButtonBindings();
  }

  private void configureButtonBindings() {
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> joystick.getRawAxis(1),
            () -> -joystick.getRawAxis(0),
            () -> -joystick.getRawAxis(2)));
  }

  Pose2d relativePose = new Pose2d(0.8, 2, new Rotation2d());

  public Command getAutonomousCommand() {
    // return new ScoreCoralCommand(drive, vision);
    // return ElevatorCommands.adjustTo(elevator, relativePose);
    // FIXME: when the autonomous command ends, robot sometimes keeps going at a random velocity continiously
    return new ScoreCoralCommand(drive, vision);
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

    Pose2d pose =
        drive
            .getPose()
            .plus(
                new Transform2d(
                    Math.cos(
                                Math.toRadians(
                                    180 - elevator.getArmEncoder() - Constants.elevatorAngle))
                            * Constants.elevatorArmLength
                        - elevator.getElevatorEncoder()
                            * Math.cos(Math.toRadians(Constants.elevatorAngle)),
                    0,
                    new Rotation2d()));

    Pose3d np =
        new Pose3d(
            pose.getX(),
            pose.getY(),
            elevator.getElevatorEncoder() * Math.sin(Math.toRadians(Constants.elevatorAngle))
                + Constants.elevatorArmLength
                    * Math.sin(
                        Math.toRadians(180 - elevator.getArmEncoder() - Constants.elevatorAngle)),
            new Rotation3d());

    Logger.recordOutput("FieldSimulation/Translation", np);

    // Logger.recordOutput("RelativePosition", new Pose2d(drive.getPose() + relativePose);
  }

  public void periodic() {
    Optional<EstimatedRobotPose> robotPose = vision.getEstimatedGlobalPose();
    if (robotPose.isPresent()) {
      drive.addVisionMeasurement(
          robotPose.get().estimatedPose.toPose2d(),
          robotPose.get().timestampSeconds,
          vision.getEstimationStdDevs());
    }
  }
}
