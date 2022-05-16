package frc.robot.shuffleboard;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.drive_train.DriveMM;
import frc.robot.commands.drive_train.DriveMMTest;
import frc.robot.commands.drive_train.TurnToAngle;
import frc.robot.commands.drive_train.TurnToAngleTest;
import frc.robot.commands.drive_train.UpdateDriveLimiters;
import frc.robot.subsystems.DriveTrain;

public class TestDriveTab extends SubsystemBase {

    private DriveTrain m_driveTrain;

    private ShuffleboardTab m_testDriveTab;

    public TestDriveTab(DriveTrain driveTrain) {

        m_testDriveTab = Shuffleboard.getTab("Test Drive");

        m_driveTrain = driveTrain;

        // Column 0
        m_testDriveTab.add("Drive kP", m_driveTrain.getPIDValue("Drive", "kP"))          .withPosition(0, 0).withSize(1, 1);
        m_testDriveTab.add("Drive kI", m_driveTrain.getPIDValue("Drive", "kI"))        .withPosition(0, 1).withSize(1, 1);
        m_testDriveTab.add("Drive kD", m_driveTrain.getPIDValue("Drive", "kD"))     .withPosition(0, 2).withSize(1, 1);
        m_testDriveTab.add("Drive kF", m_driveTrain.getPIDValue("Drive", "kF"))     .withPosition(0, 3).withSize(1, 1);
        
        m_testDriveTab.add("Drive Xin", new DriveMMTest(m_driveTrain, 0))     .withPosition(0, 4).withSize(2, 1);
        m_testDriveTab.add("Drive 100in", new DriveMM(m_driveTrain, 100))          .withPosition(0, 5).withSize(2, 1);
        m_testDriveTab.add("Drive -100in", new DriveMM(m_driveTrain, -100))        .withPosition(0, 6).withSize(2, 1);

        // Comlumn 2
        m_testDriveTab.addDoubleArray("Left Steady State", m_driveTrain::getLeftValues).withPosition(2, 4).withSize(3, 3).withWidget(BuiltInWidgets.kGraph);

        // Column 3
        m_testDriveTab.addNumber("L-Setpoint", m_driveTrain::getLeftSetPoint)          .withPosition(3, 3).withSize(1, 1);
        m_testDriveTab.addNumber("L-Error", m_driveTrain::getLeftEncoderError)          .withPosition(3, 4).withSize(1, 1);

        // Column 4
        m_testDriveTab.addNumber("L-Pos", m_driveTrain::getLeftEncoderPosition)          .withPosition(4, 3).withSize(1, 1);
        m_testDriveTab.addNumber("L-Vel", m_driveTrain::getLeftEncoderVelocity)          .withPosition(4, 4).withSize(1, 1);

        //m_testDriveTab.addNumber("Proximity", m_conveyor::getProximity).withSize(1, 1).withPosition(4, 5);

        // Column 5
        m_testDriveTab.add("Differential Drive", m_driveTrain.getDiffDrive()).withPosition(5, 3).withWidget(BuiltInWidgets.kDifferentialDrive);
        m_testDriveTab.add("Drive OL", m_driveTrain.getOpenLoopRamp())          .withPosition(5, 6).withSize(1, 1);
        // Column 6
        m_testDriveTab.add("Gyro (NavX)", m_driveTrain.getGyro()).withPosition(6, 4).withWidget(BuiltInWidgets.kGyro);

        m_testDriveTab.add("Turn OL", m_driveTrain.getOpenLoopRamp())          .withPosition(6, 6).withSize(1, 1);

        // Column 7
        m_testDriveTab.add("Max Vel", m_driveTrain.getMaxVel())          .withPosition(7, 6).withSize(1, 1);

        // Column 8
        m_testDriveTab.add("Update Limiters", new UpdateDriveLimiters(m_driveTrain))     .withPosition(8, 6).withSize(2, 1);

        // Column 9
        m_testDriveTab.addDoubleArray("Right Steady State", m_driveTrain::getRightValues).withPosition(9, 4).withSize(3, 3).withWidget(BuiltInWidgets.kGraph);

        // Column 10
        m_testDriveTab.addNumber("R-Setpoint", m_driveTrain::getRightSetPoint)          .withPosition(10, 3).withSize(1, 1);
        m_testDriveTab.addNumber("R-Error", m_driveTrain::getRightEncoderError)          .withPosition(10, 4).withSize(1, 1);

        // Column 11
        m_testDriveTab.addNumber("R-Pos", m_driveTrain::getRightEncoderPosition)          .withPosition(11, 3).withSize(1, 1);
        m_testDriveTab.addNumber("R-Vel", m_driveTrain::getRightEncoderVelocity)          .withPosition(11, 4).withSize(1, 1);

        // Column 13
        m_testDriveTab.add("Turn Xdeg", new TurnToAngleTest(m_driveTrain, 0))     .withPosition(13, 4).withSize(2, 1);
        m_testDriveTab.add("Turn 90deg", new TurnToAngle(m_driveTrain, 90))          .withPosition(13, 5).withSize(2, 1);
        m_testDriveTab.add("Turn -90deg", new TurnToAngle(m_driveTrain, -90))        .withPosition(13, 6).withSize(2, 1);

        // Column 14
        m_testDriveTab.add("Turn kP", m_driveTrain.getPIDValue("Turn", "kP"))          .withPosition(14, 0).withSize(1, 1);
        m_testDriveTab.add("Turn kI", m_driveTrain.getPIDValue("Turn", "kI"))        .withPosition(14, 1).withSize(1, 1);
        m_testDriveTab.add("Turn kD", m_driveTrain.getPIDValue("Turn", "kD"))     .withPosition(14, 2).withSize(1, 1);
        m_testDriveTab.add("Turn kF", m_driveTrain.getPIDValue("Turn", "kF"))     .withPosition(14, 3).withSize(1, 1);
    }

    @Override
    public void periodic() {
        
        

    }
    
}
