package frc.robot.subsystems;

import static frc.robot.Constants.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

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

    // Invert Drive
    private int m_invSpeed;
    private boolean m_isDriveInverted;

    // Odometry class for tracking robot pose
    private final DifferentialDriveOdometry m_odometry;

    public DriveTrain() {

        m_rightLeader = new WPI_TalonSRX(kRIGHT_LEADER);
        m_rightFollower = new WPI_TalonSRX(kRIGHT_FOLLOWER);
        m_leftLeader  = new WPI_TalonSRX(kLEFT_LEADER);
        m_leftFollower = new WPI_TalonSRX(kLEFT_FOLLOWER);
        TalonSRXConfiguration config = new TalonSRXConfiguration();

        // Factory default the talons
        m_leftLeader.configAllSettings(config);
        m_rightLeader.configAllSettings(config);
        //m_rightFollower.configAllSettings(config); ???

        // Set all the configuration values that are common across
        // both sides of the drivetrain
        config.openloopRamp = kOpenLoopRampRate;
        config.continuousCurrentLimit = kContinuousCurrent;
        config.nominalOutputForward = 0;
        config.nominalOutputReverse = 0;
        config.peakOutputForward = 1;
        config.peakOutputReverse = -1;
        config.neutralDeadband = kNeutralDeadband;
        config.motionCruiseVelocity = kMotionCruiseVelocity;
        config.motionAcceleration = kMotionAcceleration;

        config.slot0.kF = kDriveGains.kF;
        config.slot0.kP = kDriveGains.kP;
        config.slot0.kI = kDriveGains.kI;
        config.slot0.kD = kDriveGains.kD;
        config.slot0.integralZone = kDriveGains.kIzone;
        config.slot0.closedLoopPeakOutput = kDriveGains.kPeakOutput;
        config.slot0.allowableClosedloopError = 10;
        config.slot0.closedLoopPeriod = 1; // 1 ms loop

        config.slot1.kF = kTurnGains.kF;
        config.slot1.kP = kTurnGains.kP;
        config.slot1.kI = kTurnGains.kI;
        config.slot1.kD = kTurnGains.kD;
        config.slot1.allowableClosedloopError = 10;
        config.slot1.integralZone = kTurnGains.kIzone;
        config.slot1.closedLoopPeakOutput = kTurnGains.kPeakOutput;
        config.slot1.closedLoopPeriod = 1; // 1 ms

        m_leftLeader.configAllSettings(config);
        m_rightLeader.configAllSettings(config);

        // Set the followers and inverts
        m_leftLeader.setInverted(false);
        m_leftLeader.setSensorPhase(false);
        m_rightLeader.setInverted(true);
        m_rightLeader.setSensorPhase(true);
        m_rightFollower.follow(m_rightLeader);
        m_rightFollower.setInverted(InvertType.FollowMaster);
        m_leftFollower.follow(m_leftLeader);
        m_leftFollower.setInverted(InvertType.FollowMaster);
  
        // Neutral Mode to Help slow things down
        m_rightLeader.setNeutralMode(NeutralMode.Brake);
        m_leftLeader.setNeutralMode(NeutralMode.Brake);

        // Setup quadrature as primary encoder for PID driving
        m_rightLeader.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, kPID_PRIMARY, 0);
        m_leftLeader.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, kPID_PRIMARY, 0);
        m_rightLeader.setSelectedSensorPosition(0, kSlotDrive, 0);
        m_leftLeader.setSelectedSensorPosition(0, kSlotDrive, 0);
    
        // select profile slot
        m_leftLeader.selectProfileSlot(kSlotDrive, kPID_PRIMARY);
        m_rightLeader.selectProfileSlot(kSlotDrive, kPID_PRIMARY);
    
        // We have observed a couple times where the robot loses control and continues without operator
        // input, changed the TalonSRX objects to be WPI_Talons so we can use the differential drive.
        // We aren't going to actually drive with it.  We are just going to use it for the Watchdog timer.
        m_diffDrive = new DifferentialDrive(m_leftLeader, m_rightLeader);

        /* Set status frame periods */
		m_rightLeader.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, 20, kTimeoutMs);
		m_rightLeader.setStatusFramePeriod(StatusFrame.Status_14_Turn_PIDF1, 20, kTimeoutMs);
		m_leftLeader.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 5, kTimeoutMs);		//Used remotely by right Talon, speed up

        m_leftLeader.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, kTimeoutMs);
        m_leftLeader.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, kTimeoutMs);
        m_rightLeader.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, kTimeoutMs);
        m_rightLeader.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, kTimeoutMs);

    
        m_leftLeader.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 35, 40, 1.0), 0);
        m_rightLeader.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 35, 40, 1.0), 0);

        //Creates a gyro
        try{
            m_gyro = new AHRS(SerialPort.Port.kUSB1);
            m_gyro.enableLogging(true);
        } catch (RuntimeException ex) {
            DriverStation.reportError("Error instantiating navX" + ex.getMessage(), true);
        }

        // Invert Drive Variables
        m_invSpeed = 1;
        m_isDriveInverted = false;

        resetEncoders();
        m_odometry = new DifferentialDriveOdometry(m_gyro.getRotation2d());
      
        // Tuning Params //

        NetworkTable m_driveTestTable = NetworkTableInstance.getDefault().getTable("Shuffleboard").getSubTable("Drive Testing");

        m_driveAbsMaxEntry = m_driveTestTable.getEntry("Drive Max");
        m_secondsFromNeutralEntry = m_driveTestTable.getEntry("Forward Limiter");
        m_gyro.reset();

        // End of Tuning Params //

    }

    @Override
    public void periodic() {
        updateOdometry();
    }

    //  Odometry and PathWeaver methods
    public void updateOdometry() {
        m_odometry.update(m_gyro.getRotation2d(), getLeftEncoderPosition(), getRightEncoderPosition());
    }

    public Pose2d getPose() {
        return m_odometry.getPoseMeters();
    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(getLeftEncoderVelocity(), getRightEncoderVelocity());
    }

    public void resetOdometry(Pose2d pose) {
        resetEncoders();
        m_odometry.resetPosition(pose, m_gyro.getRotation2d());
    }

    public void setMaxOutput(double maxOutput) {
        m_diffDrive.setMaxOutput(maxOutput);
    }

    public DifferentialDrive getDiffDrive() {
        return m_diffDrive;
    }

    // Encoder methods
    public void resetEncoders() {
        m_leftLeader.setSelectedSensorPosition(0,kPID_PRIMARY,0);
        m_rightLeader.setSelectedSensorPosition(0,kPID_PRIMARY,0);
        m_leftLeader.setSelectedSensorPosition(0,kPID_TURN,0);
        m_rightLeader.setSelectedSensorPosition(0,kPID_TURN,0);
    }

    public double getLeftEncoderPosition() {
        return -m_leftLeader.getSelectedSensorPosition();
    }

    public double getRightEncoderPosition() {
        return m_rightLeader.getSelectedSensorPosition();
    }

    public double getLeftEncoderVelocity() {
        return -m_leftLeader.getSelectedSensorVelocity();
    }

    public double getRightEncoderVelocity() {
        return m_rightLeader.getSelectedSensorVelocity();
    }

    public double getLeftEncoderError() {
        return (getLeftSetPoint() - getLeftEncoderPosition());
    }

    public double getRightEncoderError() {
        return (getRightSetPoint() - getRightEncoderPosition());
    }

    // Gyro methods
    public double getHeading() {
        //return Math.IEEEremainder(m_gyro.getAngle(), 360);
        return m_gyro.getRotation2d().getDegrees();
    }

    public double getRawAngle() {
        return m_gyro.getAngle();
    }

    public AHRS getGyro() {
        return m_gyro;
    }

    public void resetGyro() {
        m_gyro.reset();
    }

    public double getAverageEncoderDistance() {
        return (getLeftEncoderPosition() + getRightEncoderPosition()) / 2.0;
    }

    public double getTurnRate() {
        return -m_gyro.getRate();
    }

    // Drive Train Methods
    private double deadband(double value) {
        if (Math.abs(value) >= kDriverDeadband) {
            return value;
        } else {
            return 0;
        }
    }

	@SuppressWarnings("unused")
    private double clamp(double value) {
        if (value >= m_driveAbsMax){
            return m_driveAbsMax;
        } 
        
        if (value <= -m_driveAbsMax) {
            return -m_driveAbsMax;
        }

        return value;
    }

    public void invertDrive() {

        if (m_isDriveInverted == false) {
            m_isDriveInverted = true;
        } else {
            m_isDriveInverted = false;
        }
        m_invSpeed = m_invSpeed * -1;

    }

    public void teleopDrive(double speedValue, double rotationValue) {
        m_diffDrive.arcadeDrive(deadband(m_invSpeed * speedValue), deadband(rotationValue));
    }

    public void tankDriveVolts(double leftVolts, double rightVolts) {
        m_leftLeader.setVoltage(leftVolts);
        m_rightLeader.setVoltage(rightVolts);
        feedWatchdog();
    }

    public void motionMagicStartConfigDrive(double lengthInTicks) {

        resetEncoders();

        m_leftLeader.configMotionCruiseVelocity(kMotionCruiseVelocity, kTimeoutMs);
        m_leftLeader.configMotionAcceleration(kMotionAcceleration, kTimeoutMs);
        m_rightLeader.configMotionCruiseVelocity(kMotionCruiseVelocity, kTimeoutMs);
        m_rightLeader.configMotionAcceleration(kMotionAcceleration, kTimeoutMs);

        m_leftLeader.selectProfileSlot(kSlotDrive, kPID_PRIMARY);
        m_rightLeader.selectProfileSlot(kSlotDrive, kPID_PRIMARY);

        m_leftSetpoint = lengthInTicks;
        m_rightSetpoint = lengthInTicks;
    }

    public boolean motionMagicDrive(double targetPosition) {

        m_leftLeader.set(ControlMode.MotionMagic, targetPosition);
        m_rightLeader.set(ControlMode.MotionMagic, targetPosition);

        double m_currentLetfPos = getLeftEncoderPosition();
        double m_currentRightPos = getRightEncoderPosition();

        m_diffDrive.feedWatchdog();

        return Math.abs(m_currentLetfPos - targetPosition) < kAllowableCloseLoopError && Math.abs(m_currentRightPos - targetPosition) < kAllowableCloseLoopError;
    }

    public void motionMagicStartConfigsTurn(boolean isCCWturn, double lengthInTicks){   

        resetEncoders();

		m_leftLeader.configMotionCruiseVelocity(1200, kTimeoutMs);
		m_leftLeader.configMotionAcceleration(1000, kTimeoutMs);
		m_rightLeader.configMotionCruiseVelocity(1200, kTimeoutMs);
		m_rightLeader.configMotionAcceleration(1000, kTimeoutMs);

        m_leftLeader.selectProfileSlot(kSlotTurning, kPID_TURN);
		m_rightLeader.selectProfileSlot(kSlotTurning, kPID_TURN);

        m_leftSetpoint = lengthInTicks;
        m_rightSetpoint = -lengthInTicks;
        

	}

    public boolean motionMagicTurn(double arc_in_ticks) {
    
        m_leftLeader.set(ControlMode.MotionMagic, arc_in_ticks);
		m_rightLeader.set(ControlMode.MotionMagic, -arc_in_ticks);

        double m_currentL = getLeftEncoderPosition();
        double m_currentR = getRightEncoderPosition();

        int m_targetTicks = Math.abs((int)arc_in_ticks);

        m_diffDrive.feedWatchdog();

        return (m_targetTicks - m_currentL) < kAllowableCloseLoopError && (m_targetTicks - m_currentR) < kAllowableCloseLoopError;
    }

    public void motionMagicEndConfigTurn(){

		m_leftLeader.configMotionCruiseVelocity(kMotionCruiseVelocity, kTimeoutMs);
        m_leftLeader.configMotionAcceleration(kMotionAcceleration, kTimeoutMs);
        m_rightLeader.configMotionCruiseVelocity(kMotionCruiseVelocity, kTimeoutMs);
        m_rightLeader.configMotionAcceleration(kMotionAcceleration, kTimeoutMs);

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

    public double getOpenLoopRamp() {
        return m_secondsFromNeutral;
    }

    public double getMaxVel() {
        return m_driveAbsMax;
    }

    public void feedWatchdog() {
        m_diffDrive.feed();
    }

    public double getPIDValue(String slot, String value) {
        if (slot == "Drive") {
            if (value == "Kp") {
                return kDriveGains.kP;
            } else if (value == "Ki") {
                return kDriveGains.kI;
            } else if (value == "Kd") {
                return kDriveGains.kD;
            } else if (value == "Kf") {
                return kDriveGains.kF;
            } else {
                return 0.0;
            }
        } else if (slot == "Turn"){
            if (value == "Kp") {
                return kTurnGains.kP;
            } else if (value == "Ki") {
                return kTurnGains.kI;
            } else if (value == "Kd") {
                return kTurnGains.kD;
            } else if (value == "Kf") {
                return kTurnGains.kF;
            } else {
                return 0.0;
            }
        } else {
            return 0.0;
        }

    }
    
}
