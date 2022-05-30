package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.*;

public class Conveyor  extends SubsystemBase {

    private CANSparkMax m_belt;

    public Conveyor() {
        m_belt = new CANSparkMax(kHORIZONTAL_BELT, MotorType.kBrushless);
    }

    public void belt_move(double beltValue) {
        m_belt.set(deadband(beltValue));
    }

    public void auto_belt_move(double beltValue) {
        m_belt.set(beltValue);
    }

    public void stop() {
        m_belt.set(0);
    }

    public double deadband(double value) {

        /** Upped Deadband */
        if (value >= kOperatorDeadband)
            return kBaseSpeed;
        /** Lower Deadband */
        if (value <= -kOperatorDeadband)
            return value;
        /** Outside Deadband */
        return 0;
        
    }
}