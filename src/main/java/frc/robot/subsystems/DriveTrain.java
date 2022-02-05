// RobotBuilder Version: 4.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.

// ROBOTBUILDER TYPE: Subsystem.

package frc.robot.subsystems;

//import frc.robot.commands.*;
//import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;

// END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS
// INITIALIZES MOTORS
/**
 *
 */
public class DriveTrain extends SubsystemBase {
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
    private CANSparkMax backLeftMotor;
    private CANSparkMax forwardLeftMotor;
    private MotorControllerGroup leftMotors;
    private CANSparkMax backRightMotor;
    private CANSparkMax forwardRightMotor;
    private MotorControllerGroup rightMotors;
    private DifferentialDrive robotDriver;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
    // CREATES ENCODERS
    public RelativeEncoder lEnc;
    public RelativeEncoder rEnc;

    /**
    *
    */
    public DriveTrain() {
        // CREATES MOTORS WITH ENCODERS
        backLeftMotor = new CANSparkMax(3, MotorType.kBrushless);

        backLeftMotor.restoreFactoryDefaults();
        backLeftMotor.setInverted(false);
        backLeftMotor.setIdleMode(IdleMode.kBrake);
        lEnc = backLeftMotor.getEncoder();

        forwardLeftMotor = new CANSparkMax(2, MotorType.kBrushless);

        forwardLeftMotor.restoreFactoryDefaults();
        forwardLeftMotor.setInverted(false);
        forwardLeftMotor.setIdleMode(IdleMode.kBrake);

        leftMotors = new MotorControllerGroup(forwardLeftMotor, backLeftMotor);
        addChild("LeftMotors", leftMotors);

        backRightMotor = new CANSparkMax(5, MotorType.kBrushless);

        backRightMotor.restoreFactoryDefaults();
        backRightMotor.setInverted(true);
        backRightMotor.setIdleMode(IdleMode.kBrake);

        forwardRightMotor = new CANSparkMax(4, MotorType.kBrushless);

        forwardRightMotor.restoreFactoryDefaults();
        forwardRightMotor.setInverted(true);
        forwardRightMotor.setIdleMode(IdleMode.kBrake);
        rEnc = backLeftMotor.getEncoder();

        rightMotors = new MotorControllerGroup(forwardRightMotor, backRightMotor);
        addChild("RightMotors", rightMotors);

        robotDriver = new DifferentialDrive(leftMotors, rightMotors);
        addChild("robotDriver", robotDriver);
        robotDriver.setSafetyEnabled(true);
        robotDriver.setExpiration(0.1);
        robotDriver.setMaxOutput(1.0);

    }
    // GETS POSITION OF ENCODER
    public double encoder() {
        double total = (rEnc.getPosition() + lEnc.getPosition()) / 2;
        return total * 2.1;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        // SmartDashboard.putNumber("angle", navx.getAngle());
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run when in simulation

    }

    // public double getAngle(){
    // return navx.getAngle();
    // }

    // Put methods for controlling this subsystem
    // here. Call these from Commands.
    // HOW THE ROBOT MOVES
    public void commandForDriving(double speed, double rotation) {
        robotDriver.arcadeDrive(speed, rotation);
    }

}