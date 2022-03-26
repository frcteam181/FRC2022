// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    // Robot Characteristics
    public static final double kWheelDiameterMeters = 6 * 0.0254;
    public static final double kTrackWidthMeters = 11 * 0.0254;

    public static final int kSensorUnitsPerRotation = 4096;
    public static final int kMaxRPM = 5330;
    public static final double kExpectedVelocity = (kMaxRPM/600)*kSensorUnitsPerRotation;
    public static final double kBaseSpeed = 1;
    public static final double kBaseSpeedShoot = .5;

    /**
	 * Using the configSelectedFeedbackCoefficient() function, scale units to 3600 per rotation.
	 * This is nice as it keeps 0.1 degrees of resolution, and is fairly intuitive.
	 */
	public final static double kTurnTravelUnitsPerRotation = 3600;
	
	/**
	 * Empirically measure what the difference between encoders per 360'
	 * Drive the robot in clockwise rotations and measure the units per rotation.
	 * Drive the robot in counter clockwise rotations and measure the units per rotation.
	 * Take the average of the two.
	 */
	public final static int kEncoderUnitsPerRotation = 51711; //Measure This!!!

    // TalonSRXs
    public static final int kLEFT_LEADER = 2;
    public static final int kLEFT_FOLLOWER = 1;
    public static final int kRIGHT_LEADER = 4;
    public static final int kRIGHT_FOLLOWER = 3;

    // TalonSRX configurations
    public static final int kTimeoutMs = 30;
    public static final int kPeriodMs = 10;
    public static final double kSecondsFromNeutral = 0.4;
    public static final double kNeutralDeadband = 0.001;
    public static final int kLoopTimeMs = 1;
    public static final int kAllowableCloseLoopError = 25;

    // Drive Train
    public static final double kDriveAbsMax = 0.7;
    public static final Gains kDriveGains = new Gains(0.004, 0, 0, 0, 0, 0);
    public static final Gains kTurnGains = new Gains(0, 0, 0, 0, 0, 0);
    public static final Gains kVelGains = new Gains(0, 0, 0, 0, 0, 0);
    public static final Gains kMotProfGains = new Gains(0, 0, 0, 0, 0, 0);

    // Controllers
    public static final double kDriverDeadband = 0.05;

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
