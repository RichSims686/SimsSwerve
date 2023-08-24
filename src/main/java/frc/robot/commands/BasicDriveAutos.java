package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileCommand;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.drive.Drive;

public class BasicDriveAutos {

    private static final double defaultDistanceMeters = 5;//2;  
    private static final double defaultTurnRadians = Units.rotationsToRadians(5.0);
    private static final double timeToMaxSpeed = 1.0;

    /**
     * A collectionCreates a new DriveForwardAuto, which drives forward along the x-axis a certain distance, 
     * following a trapezoidal profile with maximum speed and acceleration limits
     */
    public BasicDriveAutos() {}

    /**
     * driveForwardAuto, which drives forward along the x-axis a certain distance, 
     * following a trapezoidal profile with maximum speed and acceleration limits
     */
    public static Command driveForwardAuto(Drive drive) {
        return driveForwardAuto(defaultDistanceMeters, drive);
    }

    public static Command driveForwardAuto(double distanceMeters, Drive drive) {
        return driveForwardAuto(distanceMeters, DriveConstants.maxDriveSpeedMetersPerSec, DriveConstants.maxDriveSpeedMetersPerSec / timeToMaxSpeed, drive);
    }

    public static Command driveForwardAuto(double distanceMeters, double maxSpeedMetersPerSec, double maxAccelMetersPerSec2, Drive drive) {
        return Commands.sequence(
            new InstantCommand(drive::zeroEncoders),
            DriveLinearCommand(distanceMeters, maxSpeedMetersPerSec, maxAccelMetersPerSec2, true, drive),
            PrintEncoderDistanceCommand(drive));
    }

    /**
     * driveBackwardAuto, which drives backward along the x-axis a certain distance, 
     * following a trapezoidal profile with maximum speed and acceleration limits
     */
    public static Command driveBackwardAuto(Drive drive) {
        return driveBackwardAuto(defaultDistanceMeters, drive);
    }

    public static Command driveBackwardAuto(double distanceMeters, Drive drive) {
        return driveBackwardAuto(distanceMeters, DriveConstants.maxDriveSpeedMetersPerSec, DriveConstants.maxDriveSpeedMetersPerSec / timeToMaxSpeed, drive);
    }

    public static Command driveBackwardAuto(double distanceMeters, double maxSpeedMetersPerSec, double maxAccelMetersPerSec2, Drive drive) {
        return Commands.sequence(
            new InstantCommand(drive::zeroEncoders),
            DriveLinearCommand(distanceMeters, maxSpeedMetersPerSec, maxAccelMetersPerSec2, false, drive),
            PrintEncoderDistanceCommand(drive)); 
    }

    /**
     * driveForwardThenBackAuto, which drives forward then backward along the x-axis a certain distance, 
     * following a trapezoidal profile with maximum speed and acceleration limits
     */
    public static Command driveForwardThenBackAuto(Drive drive) {
        return driveForwardThenBackAuto(defaultDistanceMeters, drive);        
    }

    public static Command driveForwardThenBackAuto(double distanceMeters, Drive drive) {
        return driveForwardThenBackAuto(distanceMeters, DriveConstants.maxDriveSpeedMetersPerSec, DriveConstants.maxDriveSpeedMetersPerSec / timeToMaxSpeed, drive);
    }

    public static Command driveForwardThenBackAuto(double distanceMeters, double maxSpeedMetersPerSec, double maxAccelMetersPerSec2, Drive drive) {
        return Commands.sequence(
            driveForwardAuto(distanceMeters, maxSpeedMetersPerSec, maxAccelMetersPerSec2, drive),
            driveBackwardAuto(distanceMeters, maxSpeedMetersPerSec, maxAccelMetersPerSec2, drive));
    }

    /**
     * spinCcwAuto, which drives rotates CCW a certain number of radians, 
     * following a trapezoidal profile with maximum speed and acceleration limits
     */
    public static Command spinCcwAuto(Drive drive) {
        return spinCcwAuto(defaultTurnRadians, drive);
    }

    public static Command spinCcwAuto(double turnRadians, Drive drive) {
        return spinCcwAuto(turnRadians, DriveConstants.maxTurnRateRadiansPerSec, DriveConstants.maxTurnRateRadiansPerSec / timeToMaxSpeed, drive);
    }

    public static Command spinCcwAuto(double turnRadians, double maxSpeedRadPerSec, double maxAccelRadPerSec2, Drive drive) {
        return DriveRotateCommand(turnRadians, maxSpeedRadPerSec, maxAccelRadPerSec2, true, drive);
    }

    /**
     * spinCwAuto, which drives rotates CW a certain number of radians, 
     * following a trapezoidal profile with maximum speed and acceleration limits
     */
    public static Command spinCwAuto(Drive drive) {
        return spinCcwAuto(defaultTurnRadians, drive);
    }

    public static Command spinCwAuto(double turnRadians, Drive drive) {
        return spinCcwAuto(turnRadians, DriveConstants.maxTurnRateRadiansPerSec, DriveConstants.maxTurnRateRadiansPerSec / timeToMaxSpeed, drive);
    }

    public static Command spinCwAuto(double turnRadians, double maxSpeedRadPerSec, double maxAccelRadPerSec2, Drive drive) {
        return DriveRotateCommand(turnRadians, maxSpeedRadPerSec, maxAccelRadPerSec2, false, drive);
    }

    /**
     * spinCcwThenCwAuto, which spins CCW then CW certain angle, 
     * following a trapezoidal profile with maximum speed and acceleration limits
     */
    public static Command spinCcwThenCwAuto(Drive drive) {
        return spinCcwThenCwAuto(defaultTurnRadians, drive);        
    }

    public static Command spinCcwThenCwAuto(double turnRadians, Drive drive) {
        return spinCcwThenCwAuto(turnRadians, DriveConstants.maxTurnRateRadiansPerSec, DriveConstants.maxTurnRateRadiansPerSec / timeToMaxSpeed, drive);
    }

    public static Command spinCcwThenCwAuto(double turnRadians, double maxSpeedRadPerSec, double maxAccelRadPerSec2, Drive drive) {
        return Commands.sequence(
            spinCcwAuto(turnRadians, maxSpeedRadPerSec, maxAccelRadPerSec2, drive),
            spinCwAuto(turnRadians, maxSpeedRadPerSec, maxAccelRadPerSec2, drive));
    }



    // generates the TrapezoidProfileCommand for driveForwardAuto and driveBackwardAuto
    private static Command DriveLinearCommand(double distanceMeters, double maxSpeedMetersPerSec, double maxAccelMetersPerSec2, boolean forward, Drive drive) {
        double velocityMult = forward ? +1 : -1;
        return new TrapezoidProfileCommand(
            new TrapezoidProfile(
                // Limit the max acceleration and velocity
                new TrapezoidProfile.Constraints(maxSpeedMetersPerSec, maxAccelMetersPerSec2),
                // End at desired position in meters; implicitly starts at 0
                new TrapezoidProfile.State(distanceMeters, 0)),
            // Pipe the profile state to the drive
            setpointState -> drive.driveVelocity(new ChassisSpeeds(setpointState.velocity * velocityMult, 0.0, 0.0)),
            // Require the drive
            drive);
    }


    // generates the TrapezoidProfileCommand for spinCcwAuto and spinCwAuto
    private static Command DriveRotateCommand(double turnRadians, double maxSpeedRadPerSec, double maxAccelRadPerSec2, boolean ccw, Drive drive) {
        double velocityMult = ccw ? +1 : -1;
        return new TrapezoidProfileCommand(
            new TrapezoidProfile(
                // Limit the max acceleration and velocity
                new TrapezoidProfile.Constraints(maxSpeedRadPerSec, maxAccelRadPerSec2),
                // End at desired position in meters; implicitly starts at 0
                new TrapezoidProfile.State(turnRadians, 0)),
            // Pipe the profile state to the drive
            setpointState -> drive.driveVelocity(new ChassisSpeeds(0.0, 0.0, setpointState.velocity * velocityMult)),
            // Require the drive
            drive);
    }

    public static Command PrintEncoderDistanceCommand(Drive drive) {
        return new PrintFormattedCommand("Average encoder distance = %.3f radians\n" + 
            "\tWheel radius can be estimated as \n" +
            "\t(physical distance measurement / average encoder distance)\n", drive::getAverageModuleDistance);
    }
}
