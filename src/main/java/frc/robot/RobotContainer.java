// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DriveConstants.DriveModulePosition;
import frc.robot.commands.DriveWithJoysticks;
import frc.robot.commands.FeedForwardCharacterization;
import frc.robot.commands.FeedForwardCharacterization.FeedForwardCharacterizationData;
import frc.robot.commands.SpinAuto;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOFalcon500;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.leds.CANdleSystem;
import frc.robot.subsystems.leds.CANdleSystem.AnimationTypes;
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
  private final CANdleSystem candleSystem;

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

          candleSystem = new CANdleSystem(driveController.getHID());
        break;

      // Sim robot, instantiate physics sim IO implementations
      case SIM:
        drive = new Drive(
          new GyroIO() {},
          new ModuleIOSim(),
          new ModuleIOSim(),
          new ModuleIOSim(),
          new ModuleIOSim());

          candleSystem = null;
        break;

        default:
          drive = new Drive(
              new GyroIO() {},
              new ModuleIO() {},
              new ModuleIO() {},
              new ModuleIO() {},
              new ModuleIO() {});

          candleSystem = null;
    }

    // Instantiate missing subsystems
    // if (drive == null) {
    //   drive = new Drive(
    //           new GyroIO() {},
    //           new ModuleIO() {},
    //           new ModuleIO() {},
    //           new ModuleIO() {},
    //           new ModuleIO() {});
    // }

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
    if (RobotBase.isReal()) {
      // TODO: figure out how to configure buttons with CommandXboxController
      // int xButtonNum = 3;
      // int yButtonNum = 4;
      // int bButtonNum = 2;
      // new JoystickButton(driveController.getHID(), xButtonNum).whenPressed(m_candleSubsystem::setColors, m_candleSubsystem);
      // new JoystickButton(driveController.getHID(), yButtonNum).whenPressed(m_candleSubsystem::incrementAnimation, m_candleSubsystem);
      // new JoystickButton(driveController.getHID(), bButtonNum).whenPressed(m_candleSubsystem::decrementAnimation, m_candleSubsystem);
    }
  }


  private void configureSubsystems() {

    drive.setDefaultCommand(new DriveWithJoysticks(
        drive,
        () -> driveController.getLeftY(),    // forward is field +x axis
        () -> driveController.getLeftX(),    //   right is field +y axis
        () -> driveController.getRightX(),   // turn axis
        () -> !driveController.getHID().getRightBumper(),  // field relative controls
        () -> driveController.getHID().getLeftBumper()    // precision speed
        ));
  }


  private void configureAutos() {
      // Set up auto routines
      autoChooser.addDefaultOption("Do Nothing",
          new AutoRoutine(AutoPosition.ORIGIN, new InstantCommand()));

      autoChooser.addOption("Spin", new AutoRoutine(AutoPosition.ORIGIN, new SpinAuto(drive)));
                
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


  public void disabledPeriodic() {
    candleSystem.changeAnimation(AnimationTypes.Rainbow);

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

