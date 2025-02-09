package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

import edu.wpi.first.math.controller.PIDController;

public class ElevatorConstants {
    public static double innerElevatorMaximumSetpoint = 10; //FIXME


    public static Slot0Configs armConfig = new TalonFXConfiguration().Slot0
        .withKS(0)
        .withKV(0)
        .withKA(0)
        .withKP(0)
        .withKI(0)
        .withKD(0); //TODO: set motionmagic cruise vel

    public static Slot0Configs intakeConfig = new TalonFXConfiguration().Slot0
        .withKS(0)
        .withKV(0)
        .withKA(0)
        .withKP(0)
        .withKI(0)
        .withKD(0);

    public static Slot0Configs exteriorConfig = new TalonFXConfiguration().Slot0
        .withKS(0)
        .withKV(0)
        .withKA(0)
        .withKP(0)
        .withKI(0)
        .withKD(0);

    public static Slot0Configs interiorConfig = new TalonFXConfiguration().Slot0
        .withKS(0)
        .withKV(0)
        .withKA(0)
        .withKP(0)
        .withKI(0)
        .withKD(0);

}
