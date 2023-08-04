package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileCommand;
import frc.robot.subsystems.drive.Drive;

public class DriveForwardAuto extends SequentialCommandGroup  {

  private static final double distance = Units.inchesToMeters(120.0);
  private static final double maxSpeed = Units.inchesToMeters(72.0);
  private static final double timeToMaxSpeed = 1.0;
  private static final double maxAccel = maxSpeed / timeToMaxSpeed;
  private static final double rotationSpeed = 0.0;
  /**
   * Creates a new DriveForwardAuto, which drives forward along the x-axis a certain distance, 
   * following a trapezoidal profile with maximum speed and acceleration limits
   */
  public DriveForwardAuto(Drive drive) {
    super(
        // drive forward
        new TrapezoidProfileCommand(
            new TrapezoidProfile(
                // Limit the max acceleration and velocity
                new TrapezoidProfile.Constraints(maxSpeed, maxAccel),
                // End at desired position in meters; implicitly starts at 0
                new TrapezoidProfile.State(distance, 0)),
            // Pipe the profile state to the drive
            setpointState -> drive.driveVelocity(new ChassisSpeeds(setpointState.velocity, 0.0, rotationSpeed)),
            // Require the drive
            drive),

      // then drive back      
      new TrapezoidProfileCommand(
          new TrapezoidProfile(
              // Limit the max acceleration and velocity
              new TrapezoidProfile.Constraints(maxSpeed, maxAccel),
              // End at desired position in meters; implicitly starts at 0
              new TrapezoidProfile.State(distance, 0)),
          // Pipe the profile state to the drive
          setpointState -> drive.driveVelocity(new ChassisSpeeds(-setpointState.velocity, 0.0, rotationSpeed)),
          // Require the drive
          drive));
    }
}
