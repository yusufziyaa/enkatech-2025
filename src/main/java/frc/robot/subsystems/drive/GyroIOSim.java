package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.util.Units;
import frc.robot.util.PhoenixUtil;
import org.ironmaple.simulation.drivesims.GyroSimulation;
import org.littletonrobotics.junction.Logger;

public class GyroIOSim implements GyroIO {
  GyroSimulation gyroSimulation;

  public GyroIOSim(GyroSimulation gyroSimulation) {
    this.gyroSimulation = gyroSimulation;
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.yawPosition = gyroSimulation.getGyroReading();
    inputs.connected = true;
    inputs.yawVelocityRadPerSec =
        Units.degreesToRadians(gyroSimulation.getMeasuredAngularVelocity().in(RadiansPerSecond));
    Logger.recordOutput("Drive/GyroSim", gyroSimulation.getGyroReading().getDegrees());
    inputs.odometryYawPositions = gyroSimulation.getCachedGyroReadings();
    inputs.odometryYawTimestamps = PhoenixUtil.getSimulationOdometryTimeStamps();
  }
}
