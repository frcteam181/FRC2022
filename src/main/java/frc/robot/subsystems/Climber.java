package frc.robot.subsystems;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Climber extends SubsystemBase{

    VictorSP m_leftClimberPull, m_rightClimberPull;
    CANSparkMax m_leftClimberTilt, m_rightClimberTilt;
    SlewRateLimiter m_rightLimiter, m_leftLimiter;

    public Climber() {
        
        m_leftClimberPull = new VictorSP(kLEFT_CLIMBER_PULL);
        m_rightClimberPull = new VictorSP(kRIGHT_CLIMBER_PULL);

        m_leftClimberTilt = new CANSparkMax(kLEFT_CLIMBER_TILT, MotorType.kBrushless);
        m_rightClimberTilt = new CANSparkMax(kRIGHT_CLIMBER_TILT, MotorType.kBrushless);
        m_rightClimberTilt.setInverted(true);
        m_leftClimberTilt.setIdleMode(IdleMode.kBrake);
        m_rightClimberTilt.setIdleMode(IdleMode.kBrake);
        m_rightLimiter = new SlewRateLimiter(0.3);
        m_leftLimiter = new SlewRateLimiter(0.3);

    }
    
    public void extend_climberRight() {
        m_rightClimberPull.set(-kClimbSpeedRightUp);
    }

    public void retract_climberRight() {
        m_rightClimberPull.set(kClimbSpeedRightDown);
    }

    public void move_rightClimber(double rightValue) {
        m_rightClimberPull.set(clamp_climb_right(deadband(rightValue)));
    }

    public void extend_climberLeft() {
        m_leftClimberPull.set(-kClimbSpeedLeftUp);
    }

    public void retract_climberLeft() {
        m_leftClimberPull.set(kClimbSpeedLeftDown);
    }

    public void move_leftClimber(double leftValue) {
        m_leftClimberPull.set(clamp_climb_left(deadband(leftValue)));
    }

    public void stop_climber() {
        m_leftClimberPull.set(0);
        m_rightClimberPull.set(0);
    }

    public void tilt_climber_forward_withSlew() {
        m_leftClimberTilt.set(m_leftLimiter.calculate(kTiltSpeed));
        m_rightClimberTilt.set(m_rightLimiter.calculate(kTiltSpeed));
    }

    public void tilt_climber_backwards_withSlew() {
        m_leftClimberTilt.set(m_leftLimiter.calculate(-kTiltSpeed));
        m_rightClimberTilt.set(m_rightLimiter.calculate(-kTiltSpeed));
    }

    public void tilt_climber_forward() {
        m_leftClimberTilt.set(kTiltSpeed);
        m_rightClimberTilt.set(kTiltSpeed);
    }

    public void tilt_climber_backwards() {
        m_leftClimberTilt.set(-kTiltSpeed);
        m_rightClimberTilt.set(-kTiltSpeed);
    }

    public void tilt_climber_full_backwards() {
        m_leftClimberTilt.set(-0.2);
        m_rightClimberTilt.set(-0.2);
    }
    /*
     public void tilt_climber_forward_right() {
        m_rightClimberTilt.set(kTiltSpeed);
    }

     public void tilt_climber_backwards_right() {
        m_rightClimberTilt.set(-kTiltSpeed);
    }

    /*
     public void tilt_climber_forward_left() {
        m_leftClimberTilt.set(kTiltSpeed);
    }

     public void tilt_climber_backwards_left() {
        m_leftClimberTilt.set(-kTiltSpeed);
    }

    */

    public void stop_climber_tilt() {
        m_leftClimberTilt.set(0);
        m_rightClimberTilt.set(0);
    }

    public double deadband(double value) {
        /** Upped Deadband */
        if (value >= kOperatorDeadband)
            return value;
        /** Lower Deadband */
        if (value <= -kOperatorDeadband)
            return value;
        /** Outside Deadband */
        return 0;
    }

    /** Make sure the input to the set command is 1.0 >= x >= -1.0 **/
	private double clamp_climb_left(double clampValue) {
		/* Upper deadband */
		if (clampValue >= kClimbSpeedLeftUp) {
     		return kClimbSpeedLeftUp;
   		}

		/* Lower deadband */
		if (clampValue <= -kClimbSpeedLeftDown) {
      		return -kClimbSpeedLeftDown;
    	}

		/* Outside deadband */
		return clampValue;
  	}

    /** Make sure the input to the set command is 1.0 >= x >= -1.0 **/
	private double clamp_climb_right(double clampValue) {
		/* Upper deadband */
		if (clampValue >= kClimbSpeedRightUp) {
     		return kClimbSpeedRightUp;
   		}

		/* Lower deadband */
		if (clampValue <= -kClimbSpeedRightDown) {
      		return -kClimbSpeedRightDown;
    	}

		/* Outside deadband */
		return clampValue;
  	}
    
}
