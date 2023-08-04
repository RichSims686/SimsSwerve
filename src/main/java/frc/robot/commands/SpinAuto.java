package frc.robot.commands;


import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileCommand;
import frc.robot.subsystems.drive.Drive;

public class SpinAuto extends SequentialCommandGroup  {

private static final double distance = Units.rotationsToRadians(5.0); // radians
private static final double maxSpeed = Units.rotationsToRadians(1.0); // radians per sec
private static final double timeToMaxSpeed = 1.0;
private static final double maxAccel = maxSpeed / timeToMaxSpeed;
/**
   * Creates a new SpinAuto, which rotates in place for a certain number of rotations, 
   * following a trapezoidal profile with maximum rotational speed and acceleration limits
 */
public SpinAuto(Drive drive) {
  super(
      // spin CCW
      new TrapezoidProfileCommand(      
        new TrapezoidProfile(
            // Limit the max acceleration and velocity
            new TrapezoidProfile.Constraints(maxSpeed, maxAccel),
            // End at desired position in meters; implicitly starts at 0
            new TrapezoidProfile.State(distance, 0)),
        // Pipe the profile state to the drive
        setpointState -> drive.driveVelocity(new ChassisSpeeds(0.0, 0.0, setpointState.velocity)),
        // Require the drive
        drive),

      // then spin CW
      new TrapezoidProfileCommand(      
        new TrapezoidProfile(
          // Limit the max acceleration and velocity
          new TrapezoidProfile.Constraints(maxSpeed, maxAccel),
          // End at desired position in meters; implicitly starts at 0
          new TrapezoidProfile.State(distance, 0)),
      // Pipe the profile state to the drive
      setpointState -> drive.driveVelocity(new ChassisSpeeds(0.0, 0.0, -setpointState.velocity)),
      // Require the drive
      drive));
  }
}
