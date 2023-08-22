package frc.robot.subsystems.leds;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.CANdleConfiguration;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.led.LEDColor;
import frc.robot.util.led.animation.FlashingAnimation;
import frc.robot.util.led.animation.LEDAnimation;
import frc.robot.util.led.animation.LEDManager;
import frc.robot.util.led.animation.ScrollingAnimation;
import frc.robot.util.led.functions.Gradient.BasicGradient;
import frc.robot.util.led.functions.Gradient.BasicGradient.InterpolationStyle;
import frc.robot.util.led.functions.Gradient;
import frc.robot.util.led.functions.TilingFunction;
import frc.robot.util.led.strips.CANdleStrip;
import frc.robot.util.led.strips.LEDStrip.SoftwareStrip;

public class LEDFrameworkSystem extends SubsystemBase {
    private final LEDManager ledManager = LEDManager.getInstance();
    private final CANdle m_candle = new CANdle(Constants.CANDevices.candleCanID, "rio");
    private final CANdleStrip candleLEDs = new CANdleStrip(m_candle, 18*4);
    private final SoftwareStrip onboardLEDs =   candleLEDs.getOnboardLEDs();
    private final SoftwareStrip offboardLEDs =  candleLEDs.getOffboardLEDs();
    private final SoftwareStrip rightStrip =    offboardLEDs.substrip(0, 18);
    private final SoftwareStrip backStrip =     offboardLEDs.substrip(18, 36);
    private final SoftwareStrip leftStrip =     offboardLEDs.substrip(36, 54);
    private final SoftwareStrip frontStrip =    offboardLEDs.substrip(54, 72);

    private final LEDAnimation defaultOffboardAnimation = new ScrollingAnimation(Gradient.rainbow, offboardLEDs);
    private final LEDAnimation defaultOnboardAnimation = new FlashingAnimation(Gradient.blackToWhite, onboardLEDs);
    /* new ScrollingAnimation(
        new BasicGradient(
            InterpolationStyle.Step, 
            LEDColor.Blue, LEDColor.Yellow, LEDColor.Yellow, LEDColor.Red, LEDColor.Red, LEDColor.Green, LEDColor.Green, LEDColor.Blue
        ), 
        TilingFunction.Modulo, 
        offboardLEDs
    ).setVelocity(0); */

    public LEDFrameworkSystem() {
        ledManager.register(candleLEDs);
        CANdleConfiguration configAll = new CANdleConfiguration();
        configAll.statusLedOffWhenActive = true;
        configAll.disableWhenLOS = false;
        configAll.stripType = LEDStripType.GRB;
        configAll.brightnessScalar = 0.5;
        configAll.vBatOutputMode = VBatOutputMode.Modulated;
        m_candle.configFactoryDefault();
        m_candle.clearAnimation(0);
        m_candle.configAllSettings(configAll, 100);
        defaultOffboardAnimation.start();
        defaultOnboardAnimation.start();
    }

    @Override
    public void periodic() {
        ledManager.runLEDs();
    }
}
