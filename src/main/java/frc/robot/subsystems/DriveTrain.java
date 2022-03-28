package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.SensorTerm;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.*;

public class DriveTrain extends SubsystemBase{

    private WPI_TalonSRX m_leftLeader, m_leftFollower, m_rightLeader, m_rightFollower;

    private AHRS m_gyro;

    private DifferentialDrive m_diffDrive;

    // Creates a variable for Open Loop Ramp value a.k.a. Slew Rate.
    // This is for tuning purposes. After tuned, update the kSecondsFromNeutral variable on Constants.java
    private double m_secondsFromNeutral, m_driveAbsMax;
    NetworkTableEntry m_secondsFromNeutralEntry, m_driveAbsMaxEntry;

    //Motion Magic set points
    private double m_leftSetpoint, m_rightSetpoint;

    public DriveTrain() {

        //Creates a gyro
        try{
            m_gyro = new AHRS(SerialPort.Port.kUSB1);
            m_gyro.enableLogging(true);
        } catch (RuntimeException ex) {
            DriverStation.reportError("Error instantiating navX" + ex.getMessage(), true);
        }

        //Creates new TalonSRXs
        m_leftLeader = new WPI_TalonSRX(kLEFT_LEADER);
        m_leftFollower = new WPI_TalonSRX(kLEFT_FOLLOWER);
        m_rightLeader = new WPI_TalonSRX(kRIGHT_LEADER);
        m_rightFollower = new WPI_TalonSRX(kRIGHT_FOLLOWER);

        TalonSRXConfiguration m_leftConfig = new TalonSRXConfiguration();
        TalonSRXConfiguration m_rightConfig = new TalonSRXConfiguration();

        //Set follower talons to default configs, and then follow their leaders
        m_leftFollower.configAllSettings(m_leftConfig);
        m_rightFollower.configAllSettings(m_rightConfig);
        m_leftFollower.follow(m_leftLeader);
        m_leftFollower.setInverted(InvertType.FollowMaster);
        m_rightFollower.follow(m_rightLeader);
        m_rightFollower.setInverted(InvertType.FollowMaster);

        m_leftLeader.set(ControlMode.PercentOutput, 0);
        m_rightLeader.set(ControlMode.PercentOutput, 0);

        m_leftLeader.setNeutralMode(NeutralMode.Brake);
        m_rightLeader.setNeutralMode(NeutralMode.Brake);

        m_leftFollower.configOpenloopRamp(kSecondsFromNeutral, kTimeoutMs);
        m_rightFollower.configOpenloopRamp(kSecondsFromNeutral, kTimeoutMs);
        m_leftLeader.configOpenloopRamp(kSecondsFromNeutral, kTimeoutMs);
        m_rightLeader.configOpenloopRamp(kSecondsFromNeutral, kTimeoutMs);

        // Closed Loop Configuration

        /* Configure the drivetrain's left side Feedback Sensor as a Quadrature Encoder */
        m_leftLeader.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, kPID_PRIMARY, kTimeoutMs);

        /* Configure the left Talon's Selected Sensor to be a remote sensor for the right Talon */
        m_rightLeader.configRemoteFeedbackFilter(m_leftLeader.getDeviceID(), RemoteSensorSource.TalonSRX_SelectedSensor, kREMOTE_0, kTimeoutMs);

        /* Setup difference signal to be used for turn when performing Drive Straight with encoders */
		m_rightLeader.configSensorTerm(SensorTerm.Diff1, FeedbackDevice.RemoteSensor0, kTimeoutMs);	// Feedback Device of Remote Talon
		m_rightLeader.configSensorTerm(SensorTerm.Diff0, FeedbackDevice.QuadEncoder, kTimeoutMs);		// Quadrature Encoder of current Talon

        /* Difference term calculated by right Talon configured to be selected sensor of turn PID */
		m_rightLeader.configSelectedFeedbackSensor(FeedbackDevice.SensorDifference, kPID_TURN, kTimeoutMs);

        /* Scale the Feedback Sensor using a coefficient */
		/**
		 * Heading units should be scaled to ~4000 per 360 deg, due to the following limitations...
		 * - Target param for aux PID1 is 18bits with a range of [-131072,+131072] units.
		 * - Target for aux PID1 in motion profile is 14bits with a range of [-8192,+8192] units.
		 *  ... so at 3600 units per 360', that ensures 0.1 degree precision in firmware closed-loop
		 *  and motion profile trajectory points can range +-2 rotations.
		 */
		m_rightLeader.configSelectedFeedbackCoefficient(kTurnTravelUnitsPerRotation / kEncoderUnitsPerRotation,	kPID_TURN, kTimeoutMs);

        //Flip Value so when robot drives forward LEDs are green
        m_leftLeader.setInverted(true); //Update this
        m_rightLeader.setInverted(false); //Update this

        m_leftLeader.setSensorPhase(false); //Check This
        m_rightLeader.setSensorPhase(false); //Check This

        /* Set status frame periods */
		m_rightLeader.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, 20, kTimeoutMs);
		m_rightLeader.setStatusFramePeriod(StatusFrame.Status_14_Turn_PIDF1, 20, kTimeoutMs);
		m_leftLeader.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 5, kTimeoutMs);		//Used remotely by right Talon, speed up

        m_leftLeader.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, kTimeoutMs);
        m_leftLeader.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, kTimeoutMs);
        m_rightLeader.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, kTimeoutMs);
        m_rightLeader.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, kTimeoutMs);

        m_leftLeader.configNeutralDeadband(kNeutralDeadband, kTimeoutMs);
        m_rightLeader.configNeutralDeadband(kNeutralDeadband, kTimeoutMs);

        m_leftLeader.configNominalOutputForward(0, kTimeoutMs);
		m_leftLeader.configNominalOutputReverse(0, kTimeoutMs);
        m_rightLeader.configNominalOutputForward(0, kTimeoutMs);
		m_rightLeader.configNominalOutputReverse(0, kTimeoutMs);

        m_leftLeader.configPeakOutputForward(1, kTimeoutMs);
        m_leftLeader.configPeakOutputReverse(-1, kTimeoutMs);
        m_rightLeader.configPeakOutputForward(1, kTimeoutMs);
        m_rightLeader.configPeakOutputReverse(-1, kTimeoutMs);
        
        m_leftLeader.selectProfileSlot(kSlotDrive, kPID_PRIMARY);
        m_leftLeader.config_kP(kSlotDrive, kDriveGains.kP, kTimeoutMs);
        m_leftLeader.config_kI(kSlotDrive, kDriveGains.kI, kTimeoutMs);
        m_leftLeader.config_kD(kSlotDrive, kDriveGains.kD, kTimeoutMs);
        m_leftLeader.config_kF(kSlotDrive, kDriveGains.kF, kTimeoutMs);

        m_leftLeader.setSelectedSensorPosition(0, kPID_PRIMARY, kTimeoutMs);

        m_rightLeader.config_kP(kSlotTurning, kTurnGains.kP, kTimeoutMs);
        m_rightLeader.config_kI(kSlotTurning, kTurnGains.kI, kTimeoutMs);
        m_rightLeader.config_kD(kSlotTurning, kTurnGains.kD, kTimeoutMs);
        m_rightLeader.config_kF(kSlotTurning, kTurnGains.kF, kTimeoutMs);
        m_rightLeader.config_IntegralZone(kSlotTurning, kTurnGains.kIzone, kTimeoutMs);
        m_rightLeader.configClosedLoopPeakOutput(kSlotTurning, kTurnGains.kPeakOutput, kTimeoutMs);
        m_rightLeader.configAllowableClosedloopError(kSlotTurning, 0, kTimeoutMs);

        /* 1ms per loop.  PID loop can be slowed down if need be.
		 * For example,
		 * - if sensor updates are too slow
		 * - sensor deltas are very small per update, so derivative error never gets large enough to be useful.
		 * - sensor movement is very slow causing the derivative error to be near zero.
		 */
        m_rightLeader.configClosedLoopPeriod(0, kLoopTimeMs, kTimeoutMs);
        m_rightLeader.configClosedLoopPeriod(1, kLoopTimeMs, kTimeoutMs);

        /* configAuxPIDPolarity(boolean invert, int timeoutMs)
		 * false means talon's local output is PID0 + PID1, and other side Talon is PID0 - PID1
		 * true means talon's local output is PID0 - PID1, and other side Talon is PID0 + PID1
		 */
		m_rightLeader.configAuxPIDPolarity(false, kTimeoutMs);

        m_leftLeader.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 35, 40, 1.0), 0);
        m_rightLeader.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 35, 40, 1.0), 0);

        m_diffDrive = new DifferentialDrive(m_leftLeader, m_rightLeader);

        // Tuning Params //

        NetworkTable m_driveTestTable = NetworkTableInstance.getDefault().getTable("Shuffleboard").getSubTable("Drive Testing");

        m_driveAbsMaxEntry = m_driveTestTable.getEntry("Drive Max");
        m_secondsFromNeutralEntry = m_driveTestTable.getEntry("Forward Limiter");
        m_gyro.reset();

        // End of Tuning Params //

    }

    // Encoder methods
    public void resetEncoders() {
        m_leftLeader.getSensorCollection().setQuadraturePosition(0, kTimeoutMs);
        m_rightLeader.getSensorCollection().setQuadraturePosition(0, kTimeoutMs);
    }

    public double getLeftEncoderPosition() {
        return m_leftLeader.getSelectedSensorPosition();
    }

    public double getRightEncoderPosition() {
        return m_leftLeader.getSelectedSensorPosition();
    }

    // Gyro methods
    public double getHeading() {
        return Math.IEEEremainder(m_gyro.getAngle(), 360);
    }

    public double getRawAngle() {
        return m_gyro.getAngle();
    }

    public AHRS getGyro() {
        return m_gyro;
    }

    // 

    // Drive Train Methods
    private double deadband(double value) {
        if (Math.abs(value) >= kDriverDeadband) {
            return value;
        } else {
            return 0;
        }
    }

    private double clamp(double value) {
        if (value >= m_driveAbsMax){
            return m_driveAbsMax;
        } 
        
        if (value <= -m_driveAbsMax) {
            return -m_driveAbsMax;
        }

        return value;
    }

    public void teleopDrive(double speedValue, double rotationValue, boolean isSquared) {
        m_diffDrive.arcadeDrive(deadband(speedValue), deadband(-rotationValue), isSquared);
    }

    public void teleopDrive(double speedValue, double rotationValue) {
        m_diffDrive.arcadeDrive(deadband(speedValue), deadband(-rotationValue));
    }

    public void motionMagicStartConfigDrive(boolean isForward, double lengthInTicks) {

        m_leftLeader.setSafetyEnabled(false);
        m_rightLeader.setSafetyEnabled(false);

        m_leftSetpoint = getLeftEncoderPosition() + lengthInTicks;
        m_rightSetpoint = getRightEncoderPosition() + lengthInTicks;

        m_leftLeader.configMotionCruiseVelocity(1500, kTimeoutMs);
        m_leftLeader.configMotionAcceleration(750, kTimeoutMs);
        m_rightLeader.configMotionCruiseVelocity(1500, kTimeoutMs);
        m_rightLeader.configMotionAcceleration(750, kTimeoutMs);

        m_leftLeader.selectProfileSlot(kSlotDrive, kPID_PRIMARY);
        m_rightLeader.selectProfileSlot(kSlotDrive, kPID_PRIMARY);

        // if(isForward == true){
		// 	m_left_leader.config_kF(kSlot_DriveMM, kGains_Driving.kF);
		// 	m_right_leader.config_kF(kSlot_DriveMM, kGains_Driving.kF);
		// } else{
		// 	m_left_leader.config_kF(kSlot_DriveMM, kGains_Driving.kF * -1);
		// 	m_right_leader.config_kF(kSlot_DriveMM, kGains_Driving.kF * -1);
		// }
    }

    public boolean motionMagicDrive(double targetPosition) {

        m_leftLeader.set(ControlMode.MotionMagic, m_leftSetpoint);
        m_rightLeader.set(ControlMode.MotionMagic, m_rightSetpoint);

        double m_tolerance = 75;
        double m_currentLetfPos = getLeftEncoderPosition();
        double m_currentRightPos = getRightEncoderPosition();

        return Math.abs(m_currentLetfPos - m_leftSetpoint) < m_tolerance && Math.abs(m_currentRightPos - m_rightSetpoint) < m_tolerance;
    }

    public void motionMagicStartConfigsTurn(boolean isCCWturn, double lengthInTicks){

        m_leftLeader.setSafetyEnabled(false);
        m_rightLeader.setSafetyEnabled(false);

		m_leftLeader.selectProfileSlot(kSlotTurning, kPID_TURN);
		m_rightLeader.selectProfileSlot(kSlotTurning, kPID_TURN);
		m_leftLeader.configMotionCruiseVelocity(1000, kTimeoutMs);
		m_leftLeader.configMotionAcceleration(500, kTimeoutMs);
		m_rightLeader.configMotionCruiseVelocity(1000, kTimeoutMs);
		m_rightLeader.configMotionAcceleration(500, kTimeoutMs);
	
		// length in Ticks is negative
		m_leftSetpoint = getLeftEncoderPosition() + lengthInTicks;
		m_rightSetpoint = getRightEncoderPosition() - lengthInTicks;
	}

    public boolean motionMagicTurn(double degRotation) {
        
        if (Math.toRadians(degRotation) > 0) {
            m_leftLeader.set(ControlMode.MotionMagic, m_leftSetpoint);
            m_rightLeader.set(ControlMode.MotionMagic, m_rightSetpoint);
        } else {
            m_leftLeader.set(ControlMode.MotionMagic, m_leftSetpoint);
            m_rightLeader.set(ControlMode.MotionMagic, m_rightSetpoint);
        }

        double m_tolerance = 75;
        double m_currentLetfPos = getLeftEncoderPosition();
        double m_currentRightPos = getRightEncoderPosition();

        return Math.abs(m_currentLetfPos - m_leftSetpoint) < m_tolerance && Math.abs(m_currentRightPos - m_rightSetpoint) < m_tolerance;
    }

    public void motionMagicEndConfigTurn(){
		//m_leftLeader.configMotionCruiseVelocity(16636, kTimeoutMs);
		//m_leftLeader.configMotionAcceleration(8318, kTimeoutMs);
		//m_rightLeader.configMotionCruiseVelocity(16636, kTimeoutMs);
		//m_rightLeader.configMotionAcceleration(8318, kTimeoutMs);
	}

    public double getLeftSetPoint() {
        return m_leftSetpoint;
    }

    public double getRightSetPoint() {
        return m_rightSetpoint;
    }

    public void resetDrivePIDValues(double kP, double kI, double kD, double kF) {

        m_leftLeader.config_kP(kSlotDrive, kP);
        m_leftLeader.config_kI(kSlotDrive, kI);
        m_leftLeader.config_kD(kSlotDrive, kD);
        m_leftLeader.config_kF(kSlotDrive, kF);

    }

    public void resetTurnPIDValues(double kP, double kI, double kD, double kF) {

        m_leftLeader.config_kP(kSlotTurning, kP);
        m_leftLeader.config_kI(kSlotTurning, kI);
        m_leftLeader.config_kD(kSlotTurning, kD);
        m_leftLeader.config_kF(kSlotTurning, kF);

    }

    public void updateDriveLimiters() {
        m_driveAbsMax = m_driveAbsMaxEntry.getDouble(0);
        m_secondsFromNeutral = m_secondsFromNeutralEntry.getDouble(0);

        m_leftLeader.configOpenloopRamp(m_secondsFromNeutral);
        m_leftFollower.configOpenloopRamp(m_secondsFromNeutral);
        m_rightLeader.configOpenloopRamp(m_secondsFromNeutral);
        m_rightFollower.configOpenloopRamp(m_secondsFromNeutral);
    }
    
}
