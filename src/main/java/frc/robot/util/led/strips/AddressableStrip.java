package frc.robot.util.led.strips;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import frc.robot.util.led.LEDColor;
import frc.robot.util.led.LEDColor.RawLEDColor;
import frc.robot.util.led.strips.LEDStrip.HardwareStrip;

public class AddressableStrip implements HardwareStrip {
    private final AddressableLED strip;
    private final AddressableLEDBuffer buffer;

    public AddressableStrip(int PWMPort, int length) {
        this.strip = new AddressableLED(PWMPort);
        this.buffer = new AddressableLEDBuffer(length);
        this.strip.setLength(this.buffer.getLength());
        this.strip.setData(this.buffer);
        this.strip.start();
    }

    @Override
    public int getLength() {
        return buffer.getLength();
    }

    @Override
    public void setLED(int ledIndex, RawLEDColor color) {
        buffer.setRGB(ledIndex, color.r, color.g, color.b);
    }

    @Override
    public void refresh() {
        strip.setData(buffer);
    }
}
