// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.DriveConstants.DriveModulePosition;

public class Drive extends SubsystemBase {
  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private Rotation2d prevGyroYaw = new Rotation2d();

  private final Module[] modules = new Module[DriveConstants.numDriveModules]; // FL, FR, BL, BR

  private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(getModuleTranslations());

  private boolean isCharacterizing = false;
  private double characterizationVolts = 0.0;

  private ChassisSpeeds setpoint = new ChassisSpeeds();
  private SwerveModuleState[] lastSetpointStates =
      new SwerveModuleState[] {
        new SwerveModuleState(),
        new SwerveModuleState(),
        new SwerveModuleState(),
        new SwerveModuleState()
      };
  private Timer lastMovementTimer = new Timer();  // used for brake mode

  SwerveDrivePoseEstimator poseEstimator;

  private Twist2d fieldVelocity = new Twist2d();


  public Drive(GyroIO gyroIO, ModuleIO flModuleIO, ModuleIO frModuleIO, ModuleIO blModuleIO, ModuleIO brModuleIO) {
    this.gyroIO = gyroIO;
    modules[DriveModulePosition.FRONT_LEFT.ordinal()]  = new Module(flModuleIO, DriveModulePosition.FRONT_LEFT.ordinal());
    modules[DriveModulePosition.FRONT_RIGHT.ordinal()] = new Module(frModuleIO, DriveModulePosition.FRONT_RIGHT.ordinal());
    modules[DriveModulePosition.BACK_LEFT.ordinal()]   = new Module(blModuleIO, DriveModulePosition.BACK_LEFT.ordinal());
    modules[DriveModulePosition.BACK_RIGHT.ordinal()]  = new Module(brModuleIO, DriveModulePosition.BACK_RIGHT.ordinal());
    lastMovementTimer.start();
    for (var module : modules) {
      module.setBrakeMode(false);
    }

    // initialize pose estimator
    Pose2d initialPoseMeters = new Pose2d();
    poseEstimator = new SwerveDrivePoseEstimator(kinematics, getGyroRotation(), getModulePositions(), initialPoseMeters);
    prevGyroYaw = poseEstimator.getEstimatedPosition().getRotation();
  }

  public void periodic() {
    // update IO inputs
    gyroIO.updateInputs(gyroInputs);
    Logger.getInstance().processInputs("Drive/Gyro", gyroInputs);
    for (var module : modules) {
      module.periodic();
    }

    // Run modules
    if (DriverStation.isDisabled()) {
      // Stop moving while disabled
      for (var module : modules) {
        module.stop();
      }

      // Clear setpoint logs
      Logger.getInstance().recordOutput("SwerveStates/Setpoints", new double[] {});
      Logger.getInstance().recordOutput("SwerveStates/SetpointsOptimized", new double[] {});

    } else if (isCharacterizing) {
      // Run in characterization mode
      for (var module : modules) {
        module.runCharacterization(characterizationVolts);
      }

      // Clear setpoint logs
      Logger.getInstance().recordOutput("SwerveStates/Setpoints", new double[] {});
      Logger.getInstance().recordOutput("SwerveStates/SetpointsOptimized", new double[] {});

    } else {
      /**
       * Correction for swerve discrete time control issue. Borrowed from 254:
       * https://www.chiefdelphi.com/t/whitepaper-swerve-drive-skew-and-second-order-kinematics/416964/5
       */      

      // Calculate module setpoints
      Pose2d correctionPose = 
        new Pose2d(
          setpoint.vxMetersPerSecond * Constants.loopPeriodSecs,
          setpoint.vyMetersPerSecond * Constants.loopPeriodSecs,
          new Rotation2d(setpoint.omegaRadiansPerSecond * Constants.loopPeriodSecs)); // desired pose at next loop
      Twist2d correctionTwist = new Pose2d().log(correctionPose); // calculate twist from desired pose
      var correctedSpeeds =
          new ChassisSpeeds(
              correctionTwist.dx / Constants.loopPeriodSecs,
              correctionTwist.dy / Constants.loopPeriodSecs,
              correctionTwist.dtheta / Constants.loopPeriodSecs); // apply twist correction
      SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(correctedSpeeds);
      SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, DriveConstants.maxDriveSpeed);

      
      // Set to last angles if zero
      if (correctedSpeeds.vxMetersPerSecond == 0.0
          && correctedSpeeds.vyMetersPerSecond == 0.0
          && correctedSpeeds.omegaRadiansPerSecond == 0) {
        for (int i = 0; i < DriveConstants.numDriveModules; i++) {
          setpointStates[i] = new SwerveModuleState(0.0, lastSetpointStates[i].angle);
        }
      }
      lastSetpointStates = setpointStates;

      // Send setpoints to modules
      SwerveModuleState[] optimizedStates = new SwerveModuleState[DriveConstants.numDriveModules];
      for (int i = 0; i < DriveConstants.numDriveModules; i++) {
        optimizedStates[i] = modules[i].runSetpoint(setpointStates[i]);
      }

      // Log setpoint states
      Logger.getInstance().recordOutput("SwerveStates/Setpoints", setpointStates);
      Logger.getInstance().recordOutput("SwerveStates/SetpointsOptimized", optimizedStates);
    }

    // Log measured states
    SwerveModuleState[] measuredStates = new SwerveModuleState[DriveConstants.numDriveModules];
    for (int i = 0; i < DriveConstants.numDriveModules; i++) {
      measuredStates[i] = modules[i].getState();
    }
    Logger.getInstance().recordOutput("SwerveStates/Measured", measuredStates);

    // Update odometry
    Rotation2d gyroAngle;
    if (gyroInputs.connected) {
      gyroAngle = getGyroRotation();
    } else {
      // either the gyro is disconnected or we are in a simulation
      // accumulate a gyro estimate using module kinematics
      var wheelDeltas = getModulePositionDeltas();        // get change in module positions
      Twist2d twist = kinematics.toTwist2d(wheelDeltas);  // dtheta will be the estimated change in chassis angle
      gyroAngle = prevGyroYaw.plus(Rotation2d.fromRadians(twist.dtheta));
    }
    poseEstimator.update(gyroAngle, getModulePositions());
    
    Logger.getInstance().recordOutput("Odometry/Robot", getPose());

    // Update field velocity
    ChassisSpeeds chassisSpeeds = kinematics.toChassisSpeeds(measuredStates);
    Translation2d linearFieldVelocity =
        new Translation2d(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond)
            .rotateBy(getRotation());
    fieldVelocity =
        new Twist2d(
            linearFieldVelocity.getX(),
            linearFieldVelocity.getY(),
            gyroInputs.connected
                ? gyroInputs.yawVelocityRadPerSec
                : chassisSpeeds.omegaRadiansPerSecond);

    // Update brake mode
    // for (var module : modules) {
    //   module.setBrakeMode(true);
    // }

    // save values for next loop
    prevGyroYaw = gyroAngle;
  }

  /**
   * Runs the drive at the desired velocity.
   *
   * @param speeds Speeds in meters/sec
   */
  public void driveVelocity(ChassisSpeeds speeds) {
    isCharacterizing = false;
    setpoint = speeds;
    // speeds will be applied next drive.periodic()
  }

  public void drivePercent(ChassisSpeeds speeds) {
    driveVelocity(new ChassisSpeeds(
                speeds.vxMetersPerSecond * DriveConstants.maxDriveSpeed,
                speeds.vyMetersPerSecond * DriveConstants.maxDriveSpeed,
                speeds.omegaRadiansPerSecond * DriveConstants.maxTurnRate));
  }

  /** Stops the drive. */
  public void stop() {
    driveVelocity(new ChassisSpeeds());
  }

  /**
   * Stops the drive and turns the modules to an X arrangement to resist movement. The modules will
   * return to their normal orientations the next time a nonzero velocity is requested.
   */
  public void stopWithX() {
    stop();
    for (int i = 0; i < DriveConstants.numDriveModules; i++) {
      lastSetpointStates[i] =
          new SwerveModuleState(
              lastSetpointStates[i].speedMetersPerSecond, getModuleTranslations()[i].getAngle());
    }
  }

  /** Returns the maximum linear speed in meters per sec. */
  public double getMaxLinearSpeedMetersPerSec() {
    return DriveConstants.maxDriveSpeed;
  }

  /** Returns the maximum angular speed in radians per sec. */
  public double getMaxAngularSpeedRadPerSec() {
    return DriveConstants.maxTurnRate;
  }

  /**
   * Returns the measured X, Y, and theta field velocities in meters per sec. The components of the
   * twist are velocities and NOT changes in position.
   */
  public Twist2d getFieldVelocity() {
    return fieldVelocity;
  }

  /** Returns the current yaw (Z rotation). */
  public Rotation2d getGyroRotation() {
    return getYaw();
  }

  /** Returns the current yaw (Z rotation). */
  public Rotation2d getYaw() {
    return new Rotation2d(gyroInputs.yawPositionRad);
  }

  /** Returns the current pitch (Y rotation). */
  public Rotation2d getPitch() {
    return new Rotation2d(gyroInputs.pitchPositionRad);
  }

  /** Returns the current roll (X rotation). */
  public Rotation2d getRoll() {
    return new Rotation2d(gyroInputs.rollPositionRad);
  }

  /** Returns the current pitch velocity (Y rotation) in radians per second. */
  public double getPitchVelocity() {
    return gyroInputs.pitchVelocityRadPerSec;
  }

  /** Returns the current roll velocity (X rotation) in radians per second. */
  public double getRollVelocity() {
    return gyroInputs.rollVelocityRadPerSec;
  }

  /** Returns the current odometry pose. */
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  /** Returns the current odometry rotation. */
  public Rotation2d getRotation() {
    return poseEstimator.getEstimatedPosition().getRotation();
  }

  /** Resets the current odometry pose. */
  public void setPose(Pose2d pose) {
    poseEstimator.resetPosition(getGyroRotation(), getModulePositions(), pose);     
  }

  /** Adds vision data to the pose esimation. */
  // public void addVisionData(List<TimestampedVisionUpdate> visionData) {
  //   poseEstimator.addVisionData(visionData);
  // }

  /** Returns an array of module translations. */
  public Translation2d[] getModuleTranslations() {
    Translation2d[] moduleTranslations = new Translation2d[DriveConstants.numDriveModules];
    moduleTranslations[DriveModulePosition.FRONT_LEFT.ordinal()]  = new Translation2d( DriveConstants.trackWidthX / 2.0,  DriveConstants.trackWidthY / 2.0);
    moduleTranslations[DriveModulePosition.FRONT_RIGHT.ordinal()] = new Translation2d( DriveConstants.trackWidthX / 2.0, -DriveConstants.trackWidthY / 2.0);
    moduleTranslations[DriveModulePosition.BACK_LEFT.ordinal()]   = new Translation2d(-DriveConstants.trackWidthX / 2.0,  DriveConstants.trackWidthY / 2.0);
    moduleTranslations[DriveModulePosition.BACK_RIGHT.ordinal()]  = new Translation2d(-DriveConstants.trackWidthX / 2.0, -DriveConstants.trackWidthY / 2.0);
    return moduleTranslations;
  }

  /** Returns an array of module positions. */
  public SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] modulePositions = new SwerveModulePosition[DriveConstants.numDriveModules];
    for (int i = 0; i < DriveConstants.numDriveModules; i++) {
      modulePositions[i] = modules[i].getPosition();
    }
    return modulePositions;
  }

  /** Returns an array of module positions. */
  public SwerveModulePosition[] getModulePositionDeltas() {
    SwerveModulePosition[] modulePositionDeltas = new SwerveModulePosition[DriveConstants.numDriveModules];
    for (int i = 0; i < DriveConstants.numDriveModules; i++) {
      modulePositionDeltas[i] = modules[i].getPositionDelta();
    }
    return modulePositionDeltas;
  }


  /** Runs forwards at the commanded voltage. */
  public void runCharacterizationVolts(double volts) {
    isCharacterizing = true;
    characterizationVolts = volts;
  }

  /** Returns the average drive velocity in radians/sec. */
  public double getCharacterizationVelocity() {
    double driveVelocityAverage = 0.0;
    for (int i = 0; i < DriveConstants.numDriveModules; i++) {
      driveVelocityAverage += modules[i].getCharacterizationVelocity();
    }
    return driveVelocityAverage / DriveConstants.numDriveModules;
  }
}
