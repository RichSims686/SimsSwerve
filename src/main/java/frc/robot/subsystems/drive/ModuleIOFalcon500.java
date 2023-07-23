package frc.robot.subsystems.drive;

import java.security.InvalidParameterException;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderStatusFrame;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.CANDevices;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.DriveConstants.DriveModulePosition;

public class ModuleIOFalcon500 implements ModuleIO {
    private final TalonFX  driveMotor;
    private final TalonFX  turnMotor;
    private final CANCoder turnEncoder;
    private final double initialOffsetRadians;

    public ModuleIOFalcon500(DriveModulePosition position) {
        switch(position) {
            case FRONT_LEFT:
                driveMotor = new TalonFX(CANDevices.frontLeftDriveMotorID, CANDevices.driveCanBusName);
                turnMotor = new TalonFX(CANDevices.frontLeftTurnMotorID, CANDevices.driveCanBusName);
                turnEncoder = new CANCoder(CANDevices.frontLeftTurnEncoderID, CANDevices.driveCanBusName);
                driveMotor.setInverted(false);
                initialOffsetRadians = DriveConstants.frontLeftAngleOffset;
                break;
            case FRONT_RIGHT:
                driveMotor = new TalonFX(CANDevices.frontRightDriveMotorID, CANDevices.driveCanBusName);
                turnMotor = new TalonFX(CANDevices.frontRightTurnMotorID, CANDevices.driveCanBusName);
                turnEncoder = new CANCoder(CANDevices.frontRightTurnEncoderID, CANDevices.driveCanBusName);
                driveMotor.setInverted(true);
                initialOffsetRadians = DriveConstants.frontRightAngleOffset;
                break;
            case BACK_LEFT:
                driveMotor = new TalonFX(CANDevices.backLeftDriveMotorID, CANDevices.driveCanBusName);
                turnMotor = new TalonFX(CANDevices.backLeftTurnMotorID, CANDevices.driveCanBusName);
                turnEncoder = new CANCoder(CANDevices.backLeftTurnEncoderID, CANDevices.driveCanBusName);
                driveMotor.setInverted(false);
                initialOffsetRadians = DriveConstants.backLeftAngleOffset;
                break;
            case BACK_RIGHT:
                driveMotor = new TalonFX(CANDevices.backRightDriveMotorID, CANDevices.driveCanBusName);
                turnMotor = new TalonFX(CANDevices.backRightTurnMotorID, CANDevices.driveCanBusName);
                turnEncoder = new CANCoder(CANDevices.backRightTurnEncoderID, CANDevices.driveCanBusName);
                driveMotor.setInverted(true);
                initialOffsetRadians = DriveConstants.backRightAngleOffset;
                break;
            default:
                throw new InvalidParameterException("Invalid module index for ModuleIOFalcon500");
        }

        driveMotor.configFactoryDefault(1000);
        turnMotor.configFactoryDefault(1000);
        setFramePeriods(driveMotor, true);
        setFramePeriods(turnMotor, false);

        driveMotor.setNeutralMode(NeutralMode.Brake);
        turnMotor.setNeutralMode(NeutralMode.Brake);
        
        turnMotor.setInverted(true);
        
        // TODO: disable VoltageCompensation because of deadband problems???
        driveMotor.configVoltageCompSaturation(12, 1000);
        driveMotor.enableVoltageCompensation(true);
        turnMotor.configVoltageCompSaturation(12, 1000);
        turnMotor.enableVoltageCompensation(true);

        driveMotor.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 70, 80, 1));
        turnMotor.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 30, 40, 1));

        driveMotor.configNeutralDeadband(0, 1000);
        turnMotor.configNeutralDeadband(0, 1000);

        turnEncoder.configFactoryDefault(1000);
        turnEncoder.setStatusFramePeriod(CANCoderStatusFrame.SensorData, 20, 1000);
        turnEncoder.setStatusFramePeriod(CANCoderStatusFrame.VbatAndFaults, 255, 1000);
    }

    public void updateInputs(ModuleIOInputs inputs) {

        // TODO: update to latest CTRE API so we get better units
        inputs.drivePositionRad = driveMotor.getSelectedSensorPosition() / 2048.0 * 2.0 * Math.PI / DriveConstants.driveWheelGearReduction;
        inputs.driveVelocityRadPerSec = driveMotor.getSelectedSensorVelocity() / 2048 * 2.0 * Math.PI / DriveConstants.driveWheelGearReduction * 10;
        inputs.driveAppliedVolts = driveMotor.getMotorOutputVoltage();
        inputs.driveCurrentAmps = driveMotor.getStatorCurrent();

        inputs.turnPositionRad = MathUtil.angleModulus(Units.degreesToRadians(turnEncoder.getPosition())) - initialOffsetRadians;
        inputs.turnVelocityRadPerSec = turnEncoder.getVelocity();
        inputs.turnAppliedVolts = turnMotor.getMotorOutputVoltage();
        inputs.turnCurrentAmps = turnMotor.getStatorCurrent();
    }

    public void zeroEncoders() {
        turnEncoder.setPositionToAbsolute(1000);
        driveMotor.setSelectedSensorPosition(0, 0, 1000);
    }

    public void setDriveVoltage(double volts) {
        driveMotor.set(ControlMode.PercentOutput, volts/12);
    }

    public void setTurnVoltage(double volts) {
        turnMotor.set(ControlMode.PercentOutput, volts/12);
    }

    private static void setFramePeriods(TalonFX talon, boolean needMotorSensor) {
        talon.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 255, 1000);
        if (!needMotorSensor) {
           talon.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 255, 1000);
        }
        talon.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, 255, 1000);
        talon.setStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat, 255, 1000);
        talon.setStatusFramePeriod(StatusFrameEnhanced.Status_6_Misc, 255, 1000);
        talon.setStatusFramePeriod(StatusFrameEnhanced.Status_7_CommStatus, 255, 1000);
        talon.setStatusFramePeriod(StatusFrameEnhanced.Status_8_PulseWidth, 255, 1000);
        talon.setStatusFramePeriod(StatusFrameEnhanced.Status_9_MotProfBuffer, 255, 1000);
        talon.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 255, 1000);
        talon.setStatusFramePeriod(StatusFrameEnhanced.Status_11_UartGadgeteer, 255, 1000);
        talon.setStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1, 255, 1000);
        talon.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 255, 1000);
        talon.setStatusFramePeriod(StatusFrameEnhanced.Status_14_Turn_PIDF1, 255, 1000);
    }
}
