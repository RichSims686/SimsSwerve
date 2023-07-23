// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DriveConstants.DriveModulePosition;
import frc.robot.commands.DriveWithJoysticks;
import frc.robot.commands.SpinAuto;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOFalcon500;
import frc.robot.subsystems.drive.ModuleIOSim;

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

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser = new LoggedDashboardChooser<>("Auto Choices");

  // TODO: add LED and Brake switches
  // private DigitalInput brakeSwitch = new DigitalInput(DIOPorts.switch1);
  // private DigitalInput ledsSwitch = new DigitalInput(DIOPorts.switch2);

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
        break;

      // Sim robot, instantiate physics sim IO implementations
      case SIM:
        drive = new Drive(
          new GyroIO() {},
          new ModuleIOSim(),
          new ModuleIOSim(),
          new ModuleIOSim(),
          new ModuleIOSim());
        break;

        default:
          drive = new Drive(
              new GyroIO() {},
              new ModuleIO() {},
              new ModuleIO() {},
              new ModuleIO() {},
              new ModuleIO() {});
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
    autoChooser.addDefaultOption("Do Nothing", new InstantCommand());
    autoChooser.addOption("Spin", new SpinAuto(drive));
    configureAutos();
  }


  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
  }


  private void configureSubsystems() {

    drive.setDefaultCommand(new DriveWithJoysticks(
        drive,
        () -> -driveController.getLeftX(),    // x axis
        () -> -driveController.getLeftY(),    // y axis
        () -> -driveController.getRightX(),   // turn axis
        () ->driveController.getHID().getLeftBumper(),   // field relative controls
        () -> driveController.getHID().getRightBumper()   // precision speed
        ));
  }


  private void configureAutos() {
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
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

