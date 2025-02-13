package frc.robot.subsystems.exterior_elevator;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;

public class ExteriorElevatorIOTalonFX implements ExteriorElevatorIO {
    TalonFX m_motorA,m_motorB;
    MotionMagicTorqueCurrentFOC request = new MotionMagicTorqueCurrentFOC(0);
    public ExteriorElevatorIOTalonFX(int CANID1,int CANID2) {
        m_motorA = new TalonFX(CANID1,"canivore");
        m_motorB = new TalonFX(CANID2,"canivore");

        MotionMagicConfigs configs = new MotionMagicConfigs()
            .withMotionMagicAcceleration(0.1)
            .withMotionMagicCruiseVelocity(0.2);

        Slot0Configs cSlot0Configs = new Slot0Configs()
            .withGravityType(GravityTypeValue.Elevator_Static)
            .withKG(0)
            .withKP(0)
            .withKD(0);
        
        m_motorA.getConfigurator().apply(configs);
        m_motorA.getConfigurator().apply(cSlot0Configs);

        m_motorB.getConfigurator().apply(configs);
        m_motorB.getConfigurator().apply(cSlot0Configs);
    }
    @Override
    public void getToPosition(double pos) {
        m_motorA.setControl(request.withPosition(pos));
        m_motorB.setControl(request.withPosition(-pos));
    }
}
