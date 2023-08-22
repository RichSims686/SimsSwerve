package frc.robot.util.led.strips;

import com.ctre.phoenix.led.CANdle;

import frc.robot.util.led.LEDColor;
import frc.robot.util.led.LEDColor.RawLEDColor;
import frc.robot.util.led.strips.LEDStrip.HardwareStrip;

public class CANdleStrip implements HardwareStrip {
    private final CANdle candle;
    private final int length;
    private final RawLEDColor[] ledBuffer;
    private final RawLEDColor[] diffBuffer;
    private static final int LEDsPerFrame = 60;
    private int bufferPos = 0;

    public CANdleStrip(CANdle candle, int offboardStripLength) {
        this.candle = candle;
        this.length = Math.max(offboardStripLength, 0) + 8;
        this.ledBuffer = new RawLEDColor[this.length];
        this.diffBuffer = new RawLEDColor[this.length];
    }

    @Override
    public int getLength() {
        return length;
    }

    @Override
    public void setLED(int ledIndex, RawLEDColor color) {
        var curColor = ledBuffer[ledIndex];
        if(!color.equals(curColor)) {
            diffBuffer[ledIndex] = color;
        } else {
            diffBuffer[ledIndex] = null;
        }
    }

    @Override
    public void refresh() {
        /*
         * CANdles can only update a certain amount of LEDs per frame, so this will run
         */
        for(int i = 0, updatesThisFrame = 0; updatesThisFrame < LEDsPerFrame && i < length; i++, bufferPos = ++bufferPos % length) {
            var color = diffBuffer[bufferPos];
            if(color == null) continue;
            updatesThisFrame++;
            candle.setLEDs(color.r, color.g, color.b, color.w, bufferPos, 1);
            ledBuffer[bufferPos] = color;
            diffBuffer[bufferPos] = null;
        }
    }

    public SoftwareStrip getOnboardLEDs() {
        return this.substrip(0, 8);
    }

    public SoftwareStrip getOffboardLEDs() {
        return this.substrip(8);
    }
}
