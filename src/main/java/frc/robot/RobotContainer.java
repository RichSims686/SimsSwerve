// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DriveConstants.DriveModulePosition;
import frc.robot.commands.BasicDriveAutos;
import frc.robot.commands.DriveInSquare;
import frc.robot.commands.DriveStraightTrajectory;
import frc.robot.commands.DriveWithPreciseFlick;
import frc.robot.commands.FeedForwardCharacterization;
import frc.robot.commands.FeedForwardCharacterization.FeedForwardCharacterizationData;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOFalcon500;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.leds.LEDFrameworkSystem;
import frc.robot.util.Alert;
import frc.robot.util.Alert.AlertType;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;

  // Controller
  private final CommandXboxController driveController = new CommandXboxController(0);

  // LEDs
  private final LEDFrameworkSystem ledSystem;

  // Dashboard inputs
  private final LoggedDashboardChooser<AutoRoutine> autoChooser = new LoggedDashboardChooser<>("Auto Routine");

  // TODO: add LED and Brake switches
  // private DigitalInput brakeSwitch = new DigitalInput(DIOPorts.brakeSwitchPort);
  // private DigitalInput ledsSwitch = new DigitalInput(DIOPorts.ledSwitchPort);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    switch (Constants.mode) {
      // Real robot, instantiate hardware IO implementations
      case REAL:
        drive = new Drive(
          new GyroIOPigeon2(),
          new ModuleIOFalcon500(DriveModulePosition.FRONT_LEFT),
          new ModuleIOFalcon500(DriveModulePosition.FRONT_RIGHT),
          new ModuleIOFalcon500(DriveModulePosition.BACK_LEFT),
          new ModuleIOFalcon500(DriveModulePosition.BACK_RIGHT));

          ledSystem = new LEDFrameworkSystem();
        break;

      // Sim robot, instantiate physics sim IO implementations
      case SIM:
        drive = new Drive(
          new GyroIO() {},
          new ModuleIOSim(),
          new ModuleIOSim(),
          new ModuleIOSim(),
          new ModuleIOSim());

          ledSystem = null;
        break;

        default:
          drive = new Drive(
              new GyroIO() {},
              new ModuleIO() {},
              new ModuleIO() {},
              new ModuleIO() {},
              new ModuleIO() {});

          ledSystem = null;
    }

    // Configure the button bindings
    configureButtonBindings();
    
    configureSubsystems();
    
    // Set up auto routines
    configureAutos();

    // Alert if in tuning mode
    if (Constants.tuningMode) {
      new Alert("Tuning mode active, do not use in competition.", AlertType.INFO).set(true);
    }
  }


  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // if (RobotBase.isReal()) {
    //   driveController.x().onTrue(new InstantCommand(candleSystem::setColors, candleSystem).ignoringDisable(true));
    //   driveController.y().onTrue(new InstantCommand(candleSystem::incrementAnimation, candleSystem).ignoringDisable(true));
    //   driveController.b().onTrue(new InstantCommand(candleSystem::decrementAnimation, candleSystem).ignoringDisable(true));
    // }
  }


  private void configureSubsystems() {

    // drive.setDefaultCommand(new DriveWithJoysticks(
    //     drive,
    //     () -> -driveController.getLeftY(),    // forward is field +x axis
    //     () -> -driveController.getLeftX(),    //   right is field +y axis
    //     () -> -driveController.getRightX(),   // turn axis
    //     () -> !driveController.getHID().getRightBumper(),  // field relative controls
    //     () -> driveController.getHID().getLeftBumper()    // precision speed
    //     ));

    // drive.setDefaultCommand(new DriveWithJoysticksCardinal(
    //     drive,
    //     () -> -driveController.getLeftY(),    // forward is field +x axis
    //     () -> -driveController.getLeftX(),    //   right is field +y axis
    //     () -> Drive.getCardinalDirectionFromJoystick(() -> -driveController.getRightX(), () -> -driveController.getRightY()),   // turn axis
    //     () -> driveController.getHID().getLeftBumper()    // precision speed
    //     )
    //   );

    
    drive.setDefaultCommand(new DriveWithPreciseFlick(
      drive, 
      () -> -driveController.getLeftY(), 
      () -> -driveController.getLeftX(), 
      DriveWithPreciseFlick.headingFromJoystick(
        () -> -driveController.getRightX(), 
        () -> -driveController.getRightY(), 
        15, 
        0.5
      ), 
      () -> driveController.getHID().getLeftBumper())
    );
  }


  private void configureAutos() {
      // Set up auto routines
      autoChooser.addDefaultOption("Do Nothing",
          new AutoRoutine(AutoPosition.ORIGIN, new InstantCommand()));

      autoChooser.addOption("DriveStraightTraj", new AutoRoutine(AutoPosition.ORIGIN, new DriveStraightTrajectory(drive)));
      autoChooser.addOption("Drive In Square", new AutoRoutine(AutoPosition.ORIGIN, new DriveInSquare(drive)));

      autoChooser.addOption("Drive Forward", new AutoRoutine(AutoPosition.ORIGIN, BasicDriveAutos.driveForwardAuto(drive)));
      autoChooser.addOption("Drive Backward", new AutoRoutine(AutoPosition.ORIGIN, BasicDriveAutos.driveBackwardAuto(drive)));
      autoChooser.addOption("Drive Forward then Back", new AutoRoutine(AutoPosition.ORIGIN, BasicDriveAutos.driveForwardThenBackAuto(drive)));
            
      autoChooser.addOption("Spin CCW", new AutoRoutine(AutoPosition.ORIGIN, BasicDriveAutos.spinCcwAuto(drive)));
      autoChooser.addOption("Spin CW", new AutoRoutine(AutoPosition.ORIGIN, BasicDriveAutos.spinCwAuto(drive)));
      autoChooser.addOption("Spin CCW then CW", new AutoRoutine(AutoPosition.ORIGIN, BasicDriveAutos.spinCcwThenCwAuto(drive)));
            
      autoChooser.addOption(
          "Reset Odometry", new AutoRoutine(AutoPosition.ORIGIN, new InstantCommand(() -> drive.setPose(new Pose2d()))));

      autoChooser.addOption(
          "Drive Characterization",
          new AutoRoutine(AutoPosition.ORIGIN, new FeedForwardCharacterization(
              drive,
              true,
              new FeedForwardCharacterizationData("drive"),
              drive::runCharacterizationVolts,
              drive::getCharacterizationVelocity)));    
  }

  private static class AutoRoutine {
    public final AutoPosition position;
    public final Command command;

    public AutoRoutine(AutoPosition position, Command command) {
      this.position = position;
      this.command = command;
    }
  }

  public static enum AutoPosition {
    ORIGIN;

    public Pose2d getPose() {
      switch (this) {
        case ORIGIN:
          return new Pose2d();
        // other defined AutoPositions
        default:
          return new Pose2d();
      }
    }
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    AutoRoutine routine = autoChooser.get();
    drive.setPose(routine.position.getPose());
    return routine.command;
  }

  public void disabledInit() {
    // if (RobotBase.isReal()) {
    //   candleSystem.changeAnimation(AnimationTypes.Rainbow);
    // }
  }
  
  public void disabledPeriodic() {
    // if (activeAutoCommand == null || !activeAutoName.equals(autoChooser.getSelected())) {
    //     activeAutoCommand = new AutoRoutines(autoChooser.getSelected(), drive, pivot, telescope, wrist, intake);
    //     activeAutoName = autoChooser.getSelected();
    // }

    // drive.setBrakeMode(!brakeSwitch.get());
    // ledManager.setOff(ledsSwitch.get());
  }

  public void enabledInit() {
      // drive.setBrakeMode(true);
      // ledManager.setOff(false);
  }


}

