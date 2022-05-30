package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color;

public final class Constants {

    // Robot Characteristics
    public static final double kWheelDiameterMeters = Units.inchesToMeters(6);
    public static final double kTrackwidthMeters = Units.inchesToMeters(11);
    public static final int kGearRatio = 1;
    public static final int kMaxRPM = 5330;

    public static final int kCountsPerRevolution = 4096;
    public static final double kExpectedVelocity = (kMaxRPM/600) * kCountsPerRevolution;
	public final static double kTurnTravelUnitsPerRotation = 3600;
	public final static int kEncoderUnitsPerRotation = 51711; // 17598; Measure This!!!
    public static final double kEncoderTicksPerMeter = (kCountsPerRevolution * kGearRatio) / (Math.PI * kWheelDiameterMeters);
    public static final double kEncoderTicksPerDegree = kEncoderUnitsPerRotation / 360;

    // values for your robot.
    public static final double ksVolts = 1.069;
    public static final double kvVoltSecondsPerMeter = 2.9807;
    public static final double kaVoltSecondsSquaredPerMeter = 0.93873;
    public static final double kPDriveVel = 3.1519;
    public static final double kPDrivePos = 14.356;
    public static final double kDDrivePos = 1717.1;
    public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackwidthMeters);
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;

    // TalonSRX configurations
    public static final int k100msPerSecond = 10;
    public static final int kTimeoutMs = 30;
    public static final int kPeriodMs = 10;
    public static final double kNeutralDeadband = 0.001;
    public static final int kLoopTimeMs = 1;
    public static final int kAllowableCloseLoopError = 75;
    public static final double kOpenLoopRampRate = 0.5; //Slew Rate
    public static final int kContinuousCurrent = 20;
    public static final int kMotionCruiseVelocity = 1500;
    public static final int kMotionAcceleration = 750;

    // Drive Train gains and variables
    public static final double kDriveAbsMax = 1;
    public static final Gains kDriveGains = new Gains(0.04, 0, 0, 0.41, 100, 1.0);
    public static final Gains kTurnGains = new Gains(0.04,   0, 0, 0.41, 200, 1.0);
    public static final Gains kVelGains = new Gains(0,        0, 0, 0.00, 300, 1.0);
    public static final Gains kMotProfGains = new Gains(0,    0, 0, 0.45, 400, 1.0);

    // Controllers IDs and deadbands
    public static final int kDRIVER_CONTROLLER = 0;
    public static final int kOPERATOR_CONTROLLER = 1;
    public static final int kCLIMBER_CONTROLLER = 2;
    public static final double kDriverDeadband = 0.2;
    public static final double kOperatorDeadband = 0.1;

    // Intake Variables
    public static final Gains kIntakeGains = new Gains(0, 0, 0, 0, 100, 1);
    public static final double kIntakeSpeed = 1;

    // Conveyor Variables
    public static final double kBaseSpeed = 1;

    // Climber Variables
    public static final Gains kClimberGains = new Gains(0,    0, 0, 0.45, 400, 1.0);
    public static final int kCLIMBER_PID_SLOT = 0;

    // Shooter gains and variables
    public static final Gains kShooterGains = new Gains(0.1, 0, 0, 0, 100, 1);
    public static final double kBaseSpeedShoot = 0.5;
    public static final double kNearShot = 0.5;
    public static final double kMidShot = 0.6;
    public static final double kFarShot = 0.8;
    public static final double kSpitShot = 0.25;

    // Cargo Color
    public static final Color kRedCargo = new Color(0.457, 0.373, 0.167);
    public static final Color kBlueCargo = new Color(0.215, 0.441, 0.342);

    // Do not change anything after this line unless you rewire the robot and
    // update the spreadsheet!
    // Port assignments should match up with the spreadsheet here:
    // https://docs.google.com/spreadsheets/d/12oIY3QIFFsC3DQdTJzkeNdQm-VWWuzg8NxCcLUb8LFU/edit#gid=0

    // TalonSRXs (CAN ID)
    public static final int kLEFT_FOLLOWER = 1;
    public static final int kLEFT_LEADER = 2;
    public static final int kRIGHT_FOLLOWER = 3;
    public static final int kRIGHT_LEADER = 4;

    // SparkMaxs (CAN ID)
    public static final int kVERTICAL_BELT = 5;
    public static final int kHORIZONTAL_BELT = 6;
    public static final int kINTAKE = 7;
    public static final int kSHOOTER = 8;
    public static final int kCLIMBER_LEFT = 9;
    public static final int kCLIMBER_RIGHT = 10;

    // Digital Input (DIO ID)
    public static final int kTOP_SWITCH = 0;
    public static final int kMID_SWITCH = 1;
    public static final int kBOTTOM_SWITCH = 2;

    /** ---- Flat constants, you should not need to change these ---- */
	/* We allow either a 0 or 1 when selecting an ordinal for remote devices [You can have up to 2 devices assigned remotely to a talon/victor] */
	public final static int kREMOTE_0 = 0;
	public final static int kREMOTE_1 = 1;
	/* We allow either a 0 or 1 when selecting a PID Index, where 0 is primary and 1 is auxiliary */
	public final static int kPID_PRIMARY = 0;
	public final static int kPID_TURN = 1;
	/* Firmware currently supports slots [0, 3] and can be used for either PID Set */
	public final static int kSLOT_0 = 0;
	public final static int kSLOT_1 = 1;
	public final static int kSLOT_2 = 2;
	public final static int kSLOT_3 = 3;
	/* ---- Named slots, used to clarify code ---- */
	public final static int kSlotDrive = kSLOT_0;
	public final static int kSlotTurning = kSLOT_1;
	public final static int kSlotVelocit = kSLOT_2;
	public final static int kSlotMotProf = kSLOT_3;
}
