// RobotBuilder Version: 4.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.

// ROBOTBUILDER TYPE: RobotContainer.

package frc.robot;

import frc.robot.commands.*;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS
import edu.wpi.first.wpilibj2.command.Command;
//import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
//import frc.robot.subsystems.*;

// END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot
 * (including subsystems, commands, and button mappings) should be declared
 * here.
 */

// INITIALIZES CLASSES AND OBJECTS FOR THE DRIVE TRAIN AND CONTROLLER
public class RobotContainer {

  private static RobotContainer m_robotContainer = new RobotContainer();

  // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
  // The robot's subsystems
  public final DriveTrain m_driveTrain = new DriveTrain();

  // Joysticks
  private final XboxController xboxController = new XboxController(0);

  // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS

  // A chooser for autonomous commands
  SendableChooser<Command> m_chooser = new SendableChooser<>();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  private RobotContainer() {
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=SMARTDASHBOARD
    // Smartdashboard Subsystems

    // SmartDashboard Buttons
    // LINKS BUTTONS WITH COMMANDS
    SmartDashboard.putData("DriveTrainTurnLeft90DegreesCommand", new DriveTrainTurnLeft90DegreesCommand(m_driveTrain));
    SmartDashboard.putData("DriveTrainTurnRight90DegreesCommand",
        new DriveTrainTurnRight90DegreesCommand(m_driveTrain));
    SmartDashboard.putData("DriveTrainTurnAroundCommand", new DriveTrainTurnAroundCommand(m_driveTrain));
    SmartDashboard.putData("DriveTrainMoveForward5FeetCommand", new DriveTrainMoveForward5FeetCommand(m_driveTrain));
    SmartDashboard.putData("DriveTrainDefaultCommand", new DriveTrainDefaultCommand(m_driveTrain));
    SmartDashboard.putData("ForwardTurnBackCommandGroup", new ForwardTurnBackCommandGroup(m_driveTrain));
    SmartDashboard.putData("SquareCommandGroup", new SquareCommandGroup(m_driveTrain));
    SmartDashboard.putData("CircleCommand", new CircleCommand(m_driveTrain));
    SmartDashboard.putData("DriveTrainMeasuredMoveCommand", new DriveTrainMeasuredMoveCommand(m_driveTrain));

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=SMARTDASHBOARD
    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=SUBSYSTEM_DEFAULT_COMMAND
    m_driveTrain.setDefaultCommand(new DriveTrainDefaultCommand(m_driveTrain));

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=SUBSYSTEM_DEFAULT_COMMAND

    // Configure autonomous sendable chooser

    SmartDashboard.putData("Auto Mode", m_chooser);
  }

  public static RobotContainer getInstance() {
    return m_robotContainer;
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=BUTTONS
    // Create some buttons
    final JoystickButton measuredMoveButtonStart = new JoystickButton(xboxController,
        XboxController.Button.kStart.value);
    measuredMoveButtonStart.whenPressed(new DriveTrainMeasuredMoveCommand(m_driveTrain), true);
    SmartDashboard.putData("MeasuredMoveButtonStart", new DriveTrainMeasuredMoveCommand(m_driveTrain));

    final JoystickButton circleButtonBack = new JoystickButton(xboxController, XboxController.Button.kBack.value);
    circleButtonBack.whenPressed(new CircleCommand(m_driveTrain), true);
    SmartDashboard.putData("CircleButtonBack", new CircleCommand(m_driveTrain));

    final JoystickButton squareButtonB = new JoystickButton(xboxController, XboxController.Button.kB.value);
    squareButtonB.whenPressed(new SquareCommandGroup(m_driveTrain), true);
    SmartDashboard.putData("SquareButtonB", new SquareCommandGroup(m_driveTrain));

    final JoystickButton move5FeetForwardButtonY = new JoystickButton(xboxController, XboxController.Button.kY.value);
    move5FeetForwardButtonY.whenPressed(new DriveTrainMoveForward5FeetCommand(m_driveTrain), true);
    SmartDashboard.putData("Move5FeetForwardButtonY", new DriveTrainMoveForward5FeetCommand(m_driveTrain));

    final JoystickButton turnAroundButtonX = new JoystickButton(xboxController, XboxController.Button.kX.value);
    turnAroundButtonX.whenPressed(new DriveTrainTurnAroundCommand(m_driveTrain), true);
    SmartDashboard.putData("TurnAroundButtonX", new DriveTrainTurnAroundCommand(m_driveTrain));

    final JoystickButton turn90DegreesRightButtonRightTrigger = new JoystickButton(xboxController,
        XboxController.Button.kRightBumper.value);
    turn90DegreesRightButtonRightTrigger.whenPressed(new DriveTrainTurnRight90DegreesCommand(m_driveTrain), true);
    SmartDashboard.putData("Turn90DegreesRightButtonRightTrigger",
        new DriveTrainTurnRight90DegreesCommand(m_driveTrain));

    final JoystickButton turn90DegreesLeftButtonLeftTrigger = new JoystickButton(xboxController,
        XboxController.Button.kLeftBumper.value);
    turn90DegreesLeftButtonLeftTrigger.whenPressed(new DriveTrainTurnLeft90DegreesCommand(m_driveTrain), true);
    SmartDashboard.putData("Turn90DegreesLeftButtonLeftTrigger", new DriveTrainTurnLeft90DegreesCommand(m_driveTrain));

    final JoystickButton forwardTurnBackButtonA = new JoystickButton(xboxController, XboxController.Button.kA.value);
    forwardTurnBackButtonA.whenPressed(new ForwardTurnBackCommandGroup(m_driveTrain), true);
    SmartDashboard.putData("ForwardTurnBackButtonA", new ForwardTurnBackCommandGroup(m_driveTrain));

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=BUTTONS
  }

  // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=FUNCTIONS
  public XboxController getxboxController() {
    return xboxController;
  }

  // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=FUNCTIONS

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // The selected command will be run in autonomous
    return m_chooser.getSelected();
  }

}
