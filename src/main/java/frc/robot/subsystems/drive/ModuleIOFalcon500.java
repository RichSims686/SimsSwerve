package frc.robot.subsystems.drive;

import java.security.InvalidParameterException;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.Constants.CANDevices;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.DriveConstants.DriveModulePosition;

public class ModuleIOFalcon500 implements ModuleIO {
    private final TalonFX  driveMotor;
    private final TalonFX  turnMotor;
    private final CANcoder turnEncoder;
    private final double initialOffsetRadians;
    private final InvertedValue driveInverted;

    public ModuleIOFalcon500(DriveModulePosition position) {
        driveMotor = new TalonFX(position.driveMotorID, CANDevices.driveCanBusName);
        turnMotor = new TalonFX(position.turnMotorID, CANDevices.driveCanBusName);
        turnEncoder = new CANcoder(position.turnEncoderID, CANDevices.driveCanBusName);
        driveInverted = position.driveInverted;
        initialOffsetRadians = Units.rotationsToRadians(position.cancoderOffsetRotations);

        /** Configure Drive Motors */
        var driveConfig = new TalonFXConfiguration();
        // change factory defaults here
        driveConfig.MotorOutput.Inverted = driveInverted;
        driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        driveConfig.MotorOutput.DutyCycleNeutralDeadband = 0.0;
        driveConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 1;
        driveConfig.CurrentLimits.SupplyCurrentLimit = 80.0;
        driveConfig.CurrentLimits.SupplyCurrentThreshold = 70.0;
        driveConfig.CurrentLimits.SupplyTimeThreshold = 0.5;
        driveConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        driveMotor.getConfigurator().apply(driveConfig);

        /** Configure Turn Motors */
        var turnConfig = new TalonFXConfiguration();
        // change factory defaults here
        turnConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        turnConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        turnConfig.MotorOutput.DutyCycleNeutralDeadband = 0.0;        
        turnConfig.CurrentLimits.SupplyCurrentLimit = 40.0;
        turnConfig.CurrentLimits.SupplyCurrentThreshold = 30.0;
        turnConfig.CurrentLimits.SupplyTimeThreshold = 0.5;
        turnConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        turnMotor.getConfigurator().apply(turnConfig);

        setFramePeriods(driveMotor, true);
        setFramePeriods(turnMotor, false);

        zeroEncoders();

        /** Configure CANcoders */
        var cancoderConfig = new CANcoderConfiguration(); 
        // change factory defaults here
        turnEncoder.getConfigurator().apply(cancoderConfig);
        turnEncoder.getPosition().setUpdateFrequency(Constants.loopFrequencyHz);
        turnEncoder.getVelocity().setUpdateFrequency(Constants.loopFrequencyHz);
        turnEncoder.getSupplyVoltage().setUpdateFrequency(Constants.CANDevices.minCanUpdateRate);
        turnEncoder.getFaultField().setUpdateFrequency(Constants.CANDevices.minCanUpdateRate);
    }

    public void updateInputs(ModuleIOInputs inputs) {

        inputs.drivePositionRad =       Units.rotationsToRadians(driveMotor.getPosition().getValue()) / DriveConstants.driveWheelGearReduction;
        inputs.driveVelocityRadPerSec = Units.rotationsToRadians(driveMotor.getVelocity().getValue()) / DriveConstants.driveWheelGearReduction;
        inputs.driveAppliedVolts =      driveMotor.getSupplyVoltage().getValue();
        inputs.driveCurrentAmps =       driveMotor.getSupplyCurrent().getValue();

        inputs.turnPositionRad =        MathUtil.angleModulus(Units.rotationsToRadians(turnEncoder.getPosition().getValue())) - initialOffsetRadians;
        inputs.turnVelocityRadPerSec =  Units.rotationsToRadians(turnEncoder.getVelocity().getValue());
        inputs.turnAppliedVolts =       turnMotor.getSupplyVoltage().getValue();
        inputs.turnCurrentAmps =        turnMotor.getSupplyCurrent().getValue();
    }

    public void zeroEncoders() {
        driveMotor.setRotorPosition(0.0);
        turnEncoder.setPosition(turnEncoder.getAbsolutePosition().getValue());
    }

    public void setDriveVoltage(double volts) {
        driveMotor.setControl(new DutyCycleOut(volts / 12));
    }

    public void setTurnVoltage(double volts) {
        turnMotor.setControl(new DutyCycleOut(volts / 12));
    }

    private static void setFramePeriods(TalonFX talon, boolean needMotorSensor) {
        // reduce rates of most status frames

        // TODO: revisit figuring out what getters to slow down
        // talon.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 255, 1000);
        // if (!needMotorSensor) {
        //    talon.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 255, 1000);
        // }
        // talon.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, 255, 1000);
        // talon.setStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat, 255, 1000);
        // talon.setStatusFramePeriod(StatusFrameEnhanced.Status_6_Misc, 255, 1000);
        // talon.setStatusFramePeriod(StatusFrameEnhanced.Status_7_CommStatus, 255, 1000);
        // talon.setStatusFramePeriod(StatusFrameEnhanced.Status_8_PulseWidth, 255, 1000);
        // talon.setStatusFramePeriod(StatusFrameEnhanced.Status_9_MotProfBuffer, 255, 1000);
        // talon.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 255, 1000);
        // talon.setStatusFramePeriod(StatusFrameEnhanced.Status_11_UartGadgeteer, 255, 1000);
        // talon.setStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1, 255, 1000);
        // talon.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 255, 1000);
        // talon.setStatusFramePeriod(StatusFrameEnhanced.Status_14_Turn_PIDF1, 255, 1000);

        talon.getPosition().setUpdateFrequency(Constants.loopFrequencyHz);
    }
}
