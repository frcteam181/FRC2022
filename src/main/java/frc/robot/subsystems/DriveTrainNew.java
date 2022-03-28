package frc.robot.subsystems;

import static frc.robot.Constants.kEncoderUnitsPerRotation;
import static frc.robot.Constants.kGearRatio;
import static frc.robot.Constants.kNeutralDeadband;
import static frc.robot.Constants.kTimeoutMs;
import static frc.robot.Constants.kTurnTravelUnitsPerRotation;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonSRXSimCollection;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

import static frc.robot.Constants.*;

public class DriveTrainNew extends SubsystemBase {
	private final double MAX_TELEOP_DRIVE_SPEED = .70;
	// private final double arbFF = 0.2;
	// TalonFX's for the drivetrain
	// Right side is inverted here to drive forward
	WPI_TalonSRX m_left_leader, m_right_leader, m_left_follower, m_right_follower;
	// WPI_TalonFX m_left_leader, m_right_leader;//, m_left_follower, m_right_follower;

	// Variables to hold the invert types for the talons
	WPI_TalonSRX m_left_invert, m_right_invert;
	
	// Gyro 
	public AHRS m_gyro;
	
	// Auxilliary PID tracker
	// private boolean m_was_correcting = false;
	// private boolean m_is_correcting = false;

	///////////// Odometry Trackers //////////////
	// Odometry class for tracking robot pose
	private final DifferentialDriveOdometry m_odometry;
	private final Field2d m_2dField;
	
	///////////// Simulator Objects //////////////
	// These classes help us simulate our drivetrain
	public DifferentialDrivetrainSim m_drivetrainSimulator;
	/* Object for simulated inputs into Talon. */
	private TalonSRXSimCollection m_leftDriveSim, m_rightDriveSim;

	// RateLimiters to try to keep from tipping over
	SlewRateLimiter m_forward_limiter, m_rotation_limiter;
	private double m_drive_absMax;
	NetworkTableEntry m_left_output, m_right_output, m_forward_rate, m_rotation_rate, m_drive_max_entry;

	/////////// Vision PID Controllers ///////////
	PIDController m_speedPidController, m_turnPidController, m_cargoController;
	double visionDrivekP, visionDrivekI, visionDrivekD, visionTurnkP, visionTurnkI, visionTurnkD;

	// Motion Magic Setpoints for each side of the motor
	private double m_left_setpoint, m_right_setpoint;
	private boolean m_isCCWTurn;

  	/** Creates a new DriveTrain. */
 	public DriveTrainNew() {
    	m_gyro = new AHRS(SPI.Port.kMXP);
    	m_left_leader = new WPI_TalonSRX(kLEFT_LEADER);
		m_left_follower = new  WPI_TalonSRX(kLEFT_FOLLOWER);
    	m_right_leader = new WPI_TalonSRX(kRIGHT_LEADER);
		m_right_follower = new WPI_TalonSRX(kRIGHT_FOLLOWER);
	
    	/** Config Objects for motor controllers */
    	TalonSRXConfiguration _leftConfig = new TalonSRXConfiguration();
    	TalonSRXConfiguration _rightConfig = new TalonSRXConfiguration();

		// Set follower talons to default configs, and then follow their leaders
		m_left_follower.configAllSettings(_leftConfig);
		m_right_follower.configAllSettings(_rightConfig);
		m_left_follower.follow(m_left_leader);
		m_left_follower.setInverted(InvertType.FollowMaster);
		m_right_follower.follow(m_right_leader);
		m_right_follower.setInverted(InvertType.FollowMaster);

    		/* Set Neutral Mode */
		m_left_leader.setNeutralMode(NeutralMode.Brake);
		m_right_leader.setNeutralMode(NeutralMode.Brake);

		/* Configure output */
		m_left_leader.setInverted(false);
		m_right_leader.setInverted(true);
  
    	/* Configure the left Talon's selected sensor as integrated sensor */
		/* 
		 * Currently, in order to use a product-specific FeedbackDevice in configAll objects,
		 * you have to call toFeedbackType. This is a workaround until a product-specific
		 * FeedbackDevice is implemented for configSensorTerm
		 */
		_leftConfig.primaryPID.selectedFeedbackSensor = TalonSRXFeedbackDevice.QuadEncoder.toFeedbackDevice(); //Local Feedback Source
		_rightConfig.primaryPID.selectedFeedbackSensor = TalonSRXFeedbackDevice.QuadEncoder.toFeedbackDevice();

		// /* Configure the Remote (Left) Talon's selected sensor as a remote sensor for the right Talon */
		// _rightConfig.remoteFilter1.remoteSensorDeviceID = m_left_leader.getDeviceID(); //Device ID of Remote Source
		// _rightConfig.remoteFilter1.remoteSensorSource = RemoteSensorSource.TalonFX_SelectedSensor; //Remote Source Type
		
		// /* Setup difference signal to be used for turn when performing Drive Straight with encoders */
		// setRobotTurnConfigs(m_right_invert, _rightConfig);

		/* Config the neutral deadband. */
		_leftConfig.neutralDeadband = kNeutralDeadband;
		_rightConfig.neutralDeadband = kNeutralDeadband;

		/* max out the peak output (for all modes).  However you can
		 * limit the output of a given PID object with configClosedLoopPeakOutput().
		 */
		_leftConfig.peakOutputForward = +1.0;
		_leftConfig.peakOutputReverse = -1.0;
		_rightConfig.peakOutputForward = +1.0;
		_rightConfig.peakOutputReverse = -1.0;

		_rightConfig.slot0.kF = kDriveGains.kF;
		_rightConfig.slot0.kP = kDriveGains.kP;
		_rightConfig.slot0.kI = kDriveGains.kI;
		_rightConfig.slot0.kD = kDriveGains.kD;
		_rightConfig.slot0.integralZone = kDriveGains.kIzone;
		_rightConfig.slot0.closedLoopPeakOutput = kDriveGains.kPeakOutput;

		_rightConfig.slot1.kF = kTurnGains.kF;
		_rightConfig.slot1.kP = kTurnGains.kP;
		_rightConfig.slot1.kI = kTurnGains.kI;
		_rightConfig.slot1.kD = kTurnGains.kD;
		_rightConfig.slot1.integralZone = kTurnGains.kIzone;
		_rightConfig.slot1.closedLoopPeakOutput = kTurnGains.kPeakOutput;
		
		_leftConfig.slot0.kF = kDriveGains.kF;
		_leftConfig.slot0.kP = kDriveGains.kP;
		_leftConfig.slot0.kI = kDriveGains.kI;
		_leftConfig.slot0.kD = kDriveGains.kD;
		_leftConfig.slot0.integralZone = kDriveGains.kIzone;
		_leftConfig.slot0.closedLoopPeakOutput = kDriveGains.kPeakOutput;

		_leftConfig.slot1.kF = kTurnGains.kF;
		_leftConfig.slot1.kP = kTurnGains.kP;
		_leftConfig.slot1.kI = kTurnGains.kI;
		_leftConfig.slot1.kD = kTurnGains.kD;
		_leftConfig.slot1.integralZone = kTurnGains.kIzone;
		_leftConfig.slot1.closedLoopPeakOutput = kTurnGains.kPeakOutput;
			
		/* 1ms per loop.  PID loop can be slowed down if need be.
		 * For example,
		 * - if sensor updates are too slow
		 * - sensor deltas are very small per update, so derivative error never gets large enough to be useful.
		 * - sensor movement is very slow causing the derivative error to be near zero.
		 */

		int closedLoopTimeMs = 1;
		_rightConfig.slot0.closedLoopPeriod = closedLoopTimeMs;
		_rightConfig.slot1.closedLoopPeriod = closedLoopTimeMs;
		_rightConfig.slot2.closedLoopPeriod = closedLoopTimeMs;
   		_rightConfig.slot3.closedLoopPeriod = closedLoopTimeMs;
		_rightConfig.slot0.allowableClosedloopError = 25;
		_rightConfig.slot1.allowableClosedloopError = 25;
		_leftConfig.slot0.closedLoopPeriod = closedLoopTimeMs;
		_leftConfig.slot1.closedLoopPeriod = closedLoopTimeMs;
		_leftConfig.slot2.closedLoopPeriod = closedLoopTimeMs;
   		_leftConfig.slot3.closedLoopPeriod = closedLoopTimeMs;
		_leftConfig.slot0.allowableClosedloopError = 25;
		_leftConfig.slot1.allowableClosedloopError = 25;

		// _rightConfig.openloopRamp = kOpenLoopRamp;
		// _leftConfig.openloopRamp = kOpenLoopRamp;
   		 /* APPLY the config settings */
		m_left_leader.configAllSettings(_leftConfig);
		m_right_leader.configAllSettings(_rightConfig);
		m_left_leader.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 35, 40, 1.0), 0);
		m_right_leader.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 35, 40, 1.0), 0);

		/* Set status frame periods */
		// Leader Talons need faster updates 
		m_right_leader.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, 20, kTimeoutMs);
		m_right_leader.setStatusFramePeriod(StatusFrame.Status_14_Turn_PIDF1, 20, kTimeoutMs);
		m_left_leader.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 5, kTimeoutMs);		//Used remotely by right Talon, speed up
		// Followers can slow down certain status messages to reduce the can bus usage, per CTRE:
		// "Motor controllers that are followers can set Status 1 and Status 2 to 255ms(max) using setStatusFramePeriod."
		m_right_follower.setStatusFramePeriod(StatusFrame.Status_1_General, 255);
		m_right_follower.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 255);
		m_left_follower.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 255);
		m_left_follower.setStatusFramePeriod(StatusFrame.Status_1_General, 255);

		// setEncodersToZero();
		m_right_leader.setSelectedSensorPosition(0);
		m_left_leader.setSelectedSensorPosition(0);

		m_right_leader.configAllSettings(_rightConfig);
		m_left_leader.configAllSettings(_leftConfig);

		/// Odometry Tracker objects
		m_2dField = new Field2d();
		SmartDashboard.putData(m_2dField);
		m_odometry = new DifferentialDriveOdometry(m_gyro.getRotation2d());

		// Code for simulation within the DriveTrain Constructor
		if (Robot.isSimulation()) { // If our robot is simulated
			// This class simulates our drivetrain's motion around the field.
			/* Simulation model of the drivetrain */
			m_drivetrainSimulator = new DifferentialDrivetrainSim(
			  DCMotor.getCIM(2), // 2 Falcon 500s on each side of the drivetrain.
			  kGearRatio, // Standard AndyMark Gearing reduction.
			  2.1, // MOI of 2.1 kg m^2 (from CAD model).
			  104, // Mass of the robot is 26.5 kg.
			  (kWheelDiameterMeters/2), // Robot uses 3" radius (6" diameter) wheels.
			  kTrackWidthMeters, // Distance between wheels is _ meters.
	  
			  /*
			  * The standard deviations for measurement noise:
			  * x and y: 0.001 m
			  * heading: 0.001 rad
			  * l and r velocity: 0.1 m/s
			  * l and r position: 0.005 m
			  */
			  null // VecBuilder.fill(0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005) //Uncomment this
				  // line to add measurement noise.
			);
	  
			// PhysicsSim.getInstance().addTalonSRX(m_rightDrive, 0.75, 4000);
			// PhysicsSim.getInstance().addTalonSRX(m_leftDrive, 0.75, 4000);
			// Setup the Simulation input classes
			m_leftDriveSim = m_left_leader.getSimCollection();
			m_rightDriveSim = m_right_leader.getSimCollection();
		  } // end of constructor code for the simulation

		  // Setup the drive train limiting test variables
		  // Default the slew rate 3 meters per second
		  m_forward_limiter = new SlewRateLimiter(3);
		  m_rotation_limiter = new SlewRateLimiter(3);
		  m_drive_absMax = MAX_TELEOP_DRIVE_SPEED;
		  
		  NetworkTable driveTable = NetworkTableInstance.getDefault().getTable("Shuffleboard").getSubTable("Drive");
		  m_left_output =  driveTable.getEntry("Left Output");
		  m_right_output = driveTable.getEntry("Right Output");
		  NetworkTable driveTestable = NetworkTableInstance.getDefault().getTable("Shuffleboard").getSubTable("Drive Testing");

		  m_drive_max_entry = driveTestable.getEntry("Drive Max");
		  m_forward_rate = driveTestable.getEntry("Forward Limiter");
		  m_rotation_rate = driveTestable.getEntry("Rotation Limiter");
		  m_gyro.reset();

		  m_left_leader.setSafetyEnabled(false);
		  m_right_leader.setSafetyEnabled(false);

			//m_speedPidController = new PIDController(kGains_visionDrive.kP, kGains_visionDrive.kI, kGains_visionDrive.kD);
			//m_turnPidController = new PIDController(kGains_visionTurn.kP, kGains_visionTurn.kI, kGains_visionTurn.kD);
			//m_cargoController = new PIDController(kGains_visionCargo.kP, kGains_visionCargo.kI, kGains_visionCargo.kD);
			
  	}

	@Override
	public void periodic() {
		m_odometry.update(m_gyro.getRotation2d(),
                      nativeUnitsToDistanceMeters(m_left_leader.getSelectedSensorPosition()),
                      nativeUnitsToDistanceMeters(m_right_leader.getSelectedSensorPosition()));
   		m_2dField.setRobotPose(m_odometry.getPoseMeters());
	}

  	/* Zero all sensors used */
	public void setEncodersToZero() {
		m_left_leader.getSensorCollection().setQuadraturePosition(0, kTimeoutMs);
		m_right_leader.getSensorCollection().setQuadraturePosition(0, kTimeoutMs);
	}

	/** Deadband 5 percent, used on the gamepad */
	private double deadband(double value) {
		/* Upper deadband */
		if (value >= kDriverDeadband) {
      		return value;
    	}
		
		/* Lower deadband */
		if (value <= -kDriverDeadband) {
      		return value;
    	}

		/* Outside deadband */
		return 0;
  	}
  
  	/** Make sure the input to the set command is 1.0 >= x >= -1.0 **/
	private double clamp_drive(double value) {
		/* Upper deadband */
		if (value >= m_drive_absMax) {
     		return m_drive_absMax;
   		}

		/* Lower deadband */
		if (value <= -m_drive_absMax) {
      		return -m_drive_absMax;
    	}

		/* Outside deadband */
		return value;
  	}

	public void tank_drive_straight(double speed){
		m_right_leader.set(ControlMode.PercentOutput, speed);
		m_left_leader.set(ControlMode.PercentOutput, speed);
	}

	public void teleopDrive(double forward, double turn){
		forward = deadband(forward);
		turn = deadband(turn);

		forward = clamp_drive(forward);
		turn = clamp_drive(turn);

		//forward = -m_forward_limiter.calculate(forward) * m_drive_absMax;
		if(forward != 0 || turn != 0) {
			forward = m_forward_limiter.calculate(forward) * m_drive_absMax;
			turn = m_rotation_limiter.calculate(turn) * m_drive_absMax;
		}

		var speeds = DifferentialDrive.curvatureDriveIK(forward, turn, true);

		if(Robot.isSimulation()){
			// Just set the motors
			m_right_leader.set(ControlMode.PercentOutput, speeds.right);
			m_left_leader.set(ControlMode.PercentOutput, speeds.left);
			m_right_output.forceSetDouble(speeds.right);
			m_left_output.forceSetDouble(speeds.left);
			return;
		}

		// If the yaw value is zero, we should be driving straight with encoder correction,
		// otherwise, drive with the input values corrected for deadband
		// if( turn == 0 && forward != 0){
		// 	if(!m_was_correcting){
		// 		// First time we're correcting automatically, setup state
		// 		setEncodersToZero();
		// 		/* Determine which slot affects which PID */
		// 		m_right_leader.selectProfileSlot(kSlot_Turning, PID_TURN);
		// 		m_is_correcting = true;
		// 	}
			
		// 	double _targetAngle = m_right_leader.getSelectedSensorPosition(1);
				
		// 	/* Configured for percentOutput with Auxiliary PID on Integrated Sensors' Difference */
		// 	m_right_leader.set(ControlMode.PercentOutput, forward, DemandType.AuxPID, _targetAngle);
		// 	m_left_leader.follow(m_right_leader, FollowerType.AuxOutput1);

		// 	m_was_correcting = true;
		// }
		// else {
			// m_is_correcting = false;
			// m_was_correcting = false;
			
			// Just set the motors
			m_right_leader.set(ControlMode.PercentOutput, speeds.right);
			m_left_leader.set(ControlMode.PercentOutput, speeds.left);
			m_right_output.forceSetDouble(speeds.right);
			m_left_output.forceSetDouble(speeds.left);
		// }
  	}

	public boolean motionMagicDrive(double target_position) {
		double tolerance = 75;
		//add ifs if we need to set negative arbFF for going backward
		m_left_leader.set(ControlMode.MotionMagic, m_left_setpoint); //, DemandType.ArbitraryFeedForward, arbFF);
		m_right_leader.set(ControlMode.MotionMagic, m_right_setpoint);//, DemandType.ArbitraryFeedForward, arbFF);
		// m_left_leader.set(ControlMode.MotionMagic, target_position);
		// m_right_leader.set(ControlMode.MotionMagic, target_position);
	
		double currentPos_L = m_left_leader.getSelectedSensorPosition();
		double currentPos_R = m_right_leader.getSelectedSensorPosition();
	
		return Math.abs(currentPos_L - m_left_setpoint) < tolerance && Math.abs(currentPos_R - m_right_setpoint) < tolerance;
	}

  	public boolean motionMagicTurn(double arcTicks){
		double tolerance = 75;
		if(arcTicks > 0){
			m_left_leader.set(ControlMode.MotionMagic, m_left_setpoint);//, DemandType.ArbitraryFeedForward, -arbFF);
			m_right_leader.set(ControlMode.MotionMagic, m_right_setpoint);//, DemandType.ArbitraryFeedForward, arbFF);
		}else{
			m_left_leader.set(ControlMode.MotionMagic, m_left_setpoint);//, DemandType.ArbitraryFeedForward, arbFF);
			m_right_leader.set(ControlMode.MotionMagic, m_right_setpoint);//, DemandType.ArbitraryFeedForward, -arbFF);
		}
		double currentLeftPos =  m_left_leader.getSelectedSensorPosition();
		double currentRightPos = m_right_leader.getSelectedSensorPosition();
		 
		return Math.abs(m_left_setpoint - currentLeftPos) < tolerance && Math.abs(m_right_setpoint - currentRightPos) < tolerance;
	}

	public void motionMagicStartConfigDrive(boolean isForward, double lengthInTicks){
		m_left_setpoint = m_left_leader.getSelectedSensorPosition() + lengthInTicks;
		m_right_setpoint = m_right_leader.getSelectedSensorPosition() + lengthInTicks;

		m_left_leader.configMotionCruiseVelocity(1000, kTimeoutMs);
		m_left_leader.configMotionAcceleration(500, kTimeoutMs);
		m_right_leader.configMotionCruiseVelocity(1000, kTimeoutMs);
		m_right_leader.configMotionAcceleration(500, kTimeoutMs);
	
		
		//set up talon to use DriveMM slots
		m_left_leader.selectProfileSlot(kSlotDrive, kPID_PRIMARY);
		m_right_leader.selectProfileSlot(kSlotDrive, kPID_PRIMARY);
	
		// if(isForward == true){
		// 	m_left_leader.config_kF(kSlot_DriveMM, kGains_Driving.kF);
		// 	m_right_leader.config_kF(kSlot_DriveMM, kGains_Driving.kF);
		// } else{
		// 	m_left_leader.config_kF(kSlot_DriveMM, kGains_Driving.kF * -1);
		// 	m_right_leader.config_kF(kSlot_DriveMM, kGains_Driving.kF * -1);
		// }
	}

	public void motionMagicStartConfigsTurn(boolean isCCWturn, double lengthInTicks){

		m_left_leader.selectProfileSlot(kSlotTurning, kPID_PRIMARY);
		m_right_leader.selectProfileSlot(kSlotTurning, kPID_PRIMARY);
		m_left_leader.configMotionCruiseVelocity(1000, kTimeoutMs);
		m_left_leader.configMotionAcceleration(500, kTimeoutMs);
		m_right_leader.configMotionCruiseVelocity(1000, kTimeoutMs);
		m_right_leader.configMotionAcceleration(500, kTimeoutMs);
	
	
		// length in Ticks is negative
		m_left_setpoint = m_left_leader.getSelectedSensorPosition() + lengthInTicks;
		m_right_setpoint = m_right_leader.getSelectedSensorPosition() - lengthInTicks;
	}

	public void motionMagicEndConfigTurn(){
		// m_left_leader.configMotionCruiseVelocity(16636, kTimeoutMs);
		// m_left_leader.configMotionAcceleration(8318, kTimeoutMs);
		// m_right_leader.configMotionCruiseVelocity(16636, kTimeoutMs);
		// m_right_leader.configMotionAcceleration(8318, kTimeoutMs);
	}
	
	public double getLeftEncoderPosition(){
		return m_left_leader.getSelectedSensorPosition();
	}
	
	public double getRightEncoderPosition(){
		return m_right_leader.getSelectedSensorPosition();
	}

	public void resetDrivePIDValues(double kP, double kI, double kD, double kF) {
		m_left_leader.config_kP(kSlotDrive, kP);
		m_left_leader.config_kI(kSlotDrive, kI);
		m_left_leader.config_kD(kSlotDrive, kD);
		m_left_leader.config_kF(kSlotDrive, kF);
		
		m_right_leader.config_kP(kSlotDrive, kP);
		m_right_leader.config_kI(kSlotDrive, kI);
		m_right_leader.config_kD(kSlotDrive, kD);
		m_right_leader.config_kF(kSlotDrive, kF); 
	
	}

	public void resetTurnPIDValues(double kP, double kI, double kD, double kF) {
		m_left_leader.config_kP(kSlotTurning, kP);
		m_left_leader.config_kI(kSlotTurning, kI);
		m_left_leader.config_kD(kSlotTurning, kD);
		m_left_leader.config_kF(kSlotTurning, kF);
		
		m_right_leader.config_kP(kSlotTurning, kP);
		m_right_leader.config_kI(kSlotTurning, kI);
		m_right_leader.config_kD(kSlotTurning, kD);
		m_right_leader.config_kF(kSlotTurning, kF);
	}

	@Override
	public void simulationPeriodic() {
		/* Pass the robot battery voltage to the simulated Talon FXs */
		m_leftDriveSim.setBusVoltage(RobotController.getBatteryVoltage());
		m_rightDriveSim.setBusVoltage(RobotController.getBatteryVoltage());

		/*
			* CTRE simulation is low-level, so SimCollection inputs
			* and outputs are not affected by SetInverted(). Only
			* the regular user-level API calls are affected.
			*
			* WPILib expects +V to be forward.
			* Positive motor output lead voltage is ccw. We observe
			* on our physical robot that this is reverse for the
			* right motor, so negate it.
			*
			* We are hard-coding the negation of the values instead of
			* using getInverted() so we can catch a possible bug in the
			* robot code where the wrong value is passed to setInverted().
			*/
		m_drivetrainSimulator.setInputs(m_leftDriveSim.getMotorOutputLeadVoltage(),
								-m_rightDriveSim.getMotorOutputLeadVoltage());

		/*
			* Advance the model by 20 ms. Note that if you are running this
			* subsystem in a separate thread or have changed the nominal
			* timestep of TimedRobot, this value needs to match it.
			*/
		m_drivetrainSimulator.update(0.02);

		/*
			* Update all of our sensors.
			*
			* Since WPILib's simulation class is assuming +V is forward,
			* but -V is forward for the right motor, we need to negate the
			* position reported by the simulation class. Basically, we
			* negated the input, so we need to negate the output.
			*/

		/*m_leftDriveSim.setIntegratedSensorRawPosition(
						distanceToNativeUnits(
							m_drivetrainSimulator.getLeftPositionMeters()
						));
		m_leftDriveSim.setIntegratedSensorVelocity(
						velocityToNativeUnits(
							m_drivetrainSimulator.getLeftVelocityMetersPerSecond()
						));
		m_rightDriveSim.setIntegratedSensorRawPosition(
						distanceToNativeUnits(
							-m_drivetrainSimulator.getRightPositionMeters()
						));
		m_rightDriveSim.setIntegratedSensorVelocity(
						velocityToNativeUnits(
							-m_drivetrainSimulator.getRightVelocityMetersPerSecond()
						));*/

		int dev = SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]");
		SimDouble angle = new SimDouble(SimDeviceDataJNI.getSimValueHandle(dev, "Yaw"));
		angle.set(-m_drivetrainSimulator.getHeading().getDegrees());
	}

	private int distanceToNativeUnits(double positionMeters){
		double wheelRotations = positionMeters/(2 * Math.PI * (kWheelDiameterMeters / 2));
		double motorRotations = wheelRotations * kGearRatio;
		int sensorCounts = (int)(motorRotations * kCountsPerRevolution);
		return sensorCounts;
	}

	private int velocityToNativeUnits(double velocityMetersPerSecond){
		double wheelRotationsPerSecond = velocityMetersPerSecond/(2 * Math.PI * (kWheelDiameterMeters / 2));
		double motorRotationsPerSecond = wheelRotationsPerSecond * kGearRatio;
		double motorRotationsPer100ms = motorRotationsPerSecond / k100msPerSecond;
		int sensorCountsPer100ms = (int)(motorRotationsPer100ms * kCountsPerRevolution);
		return sensorCountsPer100ms;
	}

	private double nativeUnitsToDistanceMeters(double sensorCounts){
		double motorRotations = (double)sensorCounts / kCountsPerRevolution;
		double wheelRotations = motorRotations / kGearRatio;
		double positionMeters = wheelRotations * (2 * Math.PI * (kWheelDiameterMeters / 2));
	return positionMeters;
	}

	public void updateDriveLimiters() {
		m_drive_absMax = m_drive_max_entry.getDouble(0);
		m_forward_limiter = new SlewRateLimiter(m_forward_rate.getDouble(0));
		m_rotation_limiter = new SlewRateLimiter(m_rotation_rate.getDouble(0));
	}

	public double getHeading(){
		return Math.IEEEremainder(m_gyro.getAngle(), 360);
	}

	public double getRawAngle(){
		return m_gyro.getAngle();
	}

	public AHRS getGyro(){
		return m_gyro;
	}

	public Field2d getField(){
		return m_2dField;
	}

	public void resetVisionPidController() {
		m_speedPidController.reset();
		m_turnPidController.reset();
	}

	public void configureVisionDriveController(double kP, double kI, double kD) {
		m_speedPidController = new PIDController(kP, kI, kD);
	}

	public void configureVisionTurnController(double kP, double kI, double kD) {
		m_turnPidController = new PIDController(kP, kI, kD);
	}

	public void configureVisionCargoController(double kP) {
		//m_cargoController = new PIDController(kP, kGains_visionCargo.kI, kGains_visionCargo.kD);
	}

	public void visionDrive(double range,  double yaw, double goalInMeters) {
		
		double forwardspeed = 0;
		double turnspeed = 0;
		
		if (range > 0) {
			forwardspeed = -m_speedPidController.calculate(range, goalInMeters);
			turnspeed = -m_turnPidController.calculate(yaw, 0);

			NetworkTable visionTable = NetworkTableInstance.getDefault().getTable("Shuffleboard").getSubTable("Vision");
			visionTable.getEntry("forward drive speed").forceSetDouble(forwardspeed);
			visionTable.getEntry("Turn speed").forceSetDouble(turnspeed);
		}

		teleopDrive(forwardspeed, turnspeed);
	}
	
	public void cargoAim(double yaw, double forward) {
		double forwardspeed = forward;
		double turnspeed = 0;

		if (yaw != 0) {
			turnspeed = -m_cargoController.calculate(yaw, 0);
		}else {
			turnspeed = 0;
		}

		NetworkTable visionTable = NetworkTableInstance.getDefault().getTable("Shuffleboard").getSubTable("Vision");
		visionTable.getEntry("forward drive speed").forceSetDouble(forwardspeed);
		visionTable.getEntry("Turn speed").forceSetDouble(turnspeed);
		visionTable.getEntry("Cargo Yaw").forceSetDouble(yaw);
		teleopDrive(forwardspeed, turnspeed);
	}

	public double getAverageClosedLoopError(){
		return (m_right_leader.getClosedLoopError() + m_left_leader.getClosedLoopError()) / 2;
	}

	public double getRightSetPoint(){
		return m_right_setpoint;
	}

	public double getLeftSetPoint(){
		return m_left_setpoint;
	}
}
