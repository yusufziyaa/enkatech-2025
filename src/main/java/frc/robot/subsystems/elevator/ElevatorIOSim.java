package frc.robot.subsystems.elevator;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class ElevatorIOSim implements ElevatorIO {
  // public Mechanism2d mech = new Mechanism2d(3,3);

  Mechanism2d mech = new Mechanism2d(3, 3);
  // MechanismRoot2d root = mech.getRoot("arm", 2 - 0.885 - 0.208955, 0);
  MechanismRoot2d root = mech.getRoot("arm", 1.0869 - 0.154922 + 0.208955, 0);

  MechanismLigament2d m_elevator;
  // MechanismLigament2d mavi_parca; mavi parcayi eklemek yerine asansor daha
  // ilerideymis gibi
  // yapiyoruz
  MechanismLigament2d m_arm;

  double armAngle = 20; // deg
  double elevatorLength = 1; // m
  double intakeAngle;

  public ElevatorIOSim() {
    m_elevator =
        root.append(
            new MechanismLigament2d(
                "elevator", elevatorLength, Constants.elevatorAngle)); // asansör
    m_arm =
        m_elevator.append(new MechanismLigament2d("arm", Constants.elevatorArmLength, armAngle));
  }

  double elevatorKp = 0.1;
  double armKp = 0.1;
  double intakeKp = 0.1;

  double elevatorMaxSpeed = 10;
  double intakeMaxSpeed = 5;
  double armMaxSpeed = 5;

  @Override
  public void runVelocity(double elevatorSpeed, double armSpeed, double intakeSpeed) {
    m_arm.setAngle(getArmEncoder() + armKp * armSpeed);
    m_elevator.setLength(getElevatorEncoder() + elevatorKp * elevatorSpeed);
  }

  @Override
  public void adjustSetpoints(
      double desiredArmAngle, double desiredElevatorLength, double desiredIntakeAngle) {
    // FIXME

    m_arm.setAngle(
        getArmEncoder() + armKp * -1 * (getArmEncoder() - desiredArmAngle) * armMaxSpeed);
    m_elevator.setLength(
        getElevatorEncoder()
            + elevatorKp * (getElevatorEncoder() - desiredElevatorLength) * elevatorMaxSpeed);

    // intake not ready yet
  }

  public void addTargetMech(Pose2d pose) {
    root.append(
        new MechanismLigament2d(
            "target",
            Math.sqrt(pose.getX() * pose.getX() + pose.getY() * pose.getY()),
            Math.toDegrees(Math.atan2(pose.getY(), pose.getX()))));
  }

  @Override
  public double getElevatorEncoder() {
    return m_elevator.getLength();
  }

  @Override
  public double getArmEncoder() {
    return m_arm.getAngle();
  }

  @Override
  public double getIntakeEncoder() {
    // FIXME
    return 0;
  }

  @Override
  public void simulationPeriodic() {
    SmartDashboard.putData("Mechanism", mech);
  }

  @Override
  public void updateInputs(ElevatorInputs inputs) {
    inputs.interiorElevatorConnected = true;
    inputs.exteriorElevatorConnected = true;

    inputs.exteriorElevatorState = 0;
    inputs.interiorElevatorState = 0;
  }
}
