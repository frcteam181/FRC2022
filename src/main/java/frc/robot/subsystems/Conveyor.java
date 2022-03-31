package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
//import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.*;

public class Conveyor  extends SubsystemBase{

    private CANSparkMax m_vertticalBelt, m_horizontalBelt;
    private CANSparkMax m_shooter; 

    //private SparkMaxPIDController m_verticalBeltPID, m_horizontalBeltPID;

    public Conveyor() {
        m_vertticalBelt = new CANSparkMax(kVERTICAL_BELT, MotorType.kBrushless);
        m_horizontalBelt = new CANSparkMax(kHORIZONTAL_BELT, MotorType.kBrushless);
        m_shooter = new CANSparkMax(kSHOOTER, MotorType.kBrushless);
    }

    public void horizontalBelt_move(double horizontalValue) {
        m_horizontalBelt.set(deadband(horizontalValue));
    }

    public void verticalBelt_move(double verticalValue) {
        m_vertticalBelt.set(deadband(verticalValue));
    }

    public void shooter_move(double shooterValue) {
        m_shooter.set(deadband2(shooterValue));
    }

    public void belt_move(double beltValue) {
        m_horizontalBelt.set(beltValue);
        m_vertticalBelt.set(beltValue);
        m_shooter.set(beltValue);
    }

    public void auto_belt_move(double beltValue) {
        m_horizontalBelt.set(beltValue);
        m_vertticalBelt.set(beltValue);
    }

    public void stop() {
        m_horizontalBelt.set(0);
        m_vertticalBelt.set(0);
        m_shooter.set(0);
    }
    
      /*  public double deadband(double value) {
        // Upped Deadband 
        if (value >= kOperatorDeadband)
            return value;
        // Lower Deadband 
        if (value <= -kOperatorDeadband)
            return value;
        // Outside Deadband 
        return 0;
    } 
    */
    

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

    public double deadband2(double value) {
        /** Upped Deadband */
        if (value >= kOperatorDeadband)
            return kBaseSpeedShoot;
        /** Lower Deadband */
        if (value <= -kOperatorDeadband)
            return value;
        /** Outside Deadband */
        return 0;
    }
    
}
