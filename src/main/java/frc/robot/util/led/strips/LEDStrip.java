package frc.robot.util.led.strips;

import java.util.function.IntConsumer;

import edu.wpi.first.math.MathUtil;
import frc.robot.util.led.LEDColor;
import frc.robot.util.led.LEDColor.RawLEDColor;

public interface LEDStrip {
    public interface HardwareStrip extends LEDStrip {
        public void refresh();
    }
    public int getLength();
    public static int getLength(LEDStrip[] strips) {
        int accumLength = 0;
        for(LEDStrip strip : strips) {
            accumLength += strip.getLength();
        }
        return accumLength;
    }

    public default SoftwareStrip concat(LEDStrip... strips) {
        LEDStrip[] newStrips = new LEDStrip[strips.length + 1];
        newStrips[0] = this;
        for(int i = 0; i < strips.length; i++) {
            newStrips[i + 1] = strips[i];
        }
        return new SoftwareStrip(newStrips);
    }
    public default SoftwareStrip substrip(int startIndex) {
        return substrip(startIndex, getLength());
    }
    public default SoftwareStrip substrip(int startIndex, int endIndex) {
        return new SoftwareStrip(new LEDStrip[]{this}, startIndex, endIndex);
    }
    public default SoftwareStrip reverse() {
        return new SoftwareStrip(new LEDStrip[]{this}, true);
    }

    public default void setLED(int ledIndex, LEDColor color) {setLED(ledIndex, color.getLEDColor());}
    public void setLED(int ledIndex, RawLEDColor color);
    public default void foreach(IntConsumer function) {
        for (int i = 0; i < getLength(); i++) {
            function.accept(i);
        }
    }
    public default void clear() {
        foreach((int i) -> setLED(i, new RawLEDColor(0, 0, 0, 0)));
    }

    public static class SoftwareStrip implements LEDStrip {
        private final LEDStrip[] strips;
        public final int startIndex;
        public final int endIndex;
        private final boolean reversed;

        public SoftwareStrip(LEDStrip... strips) {
            this(strips, false);
        }
        public SoftwareStrip(LEDStrip[] strips, boolean reversed) {
            this(strips, 0, reversed);
        }
        public SoftwareStrip(LEDStrip[] strips, int startIndex) {
            this(strips, startIndex, false);
        }
        public SoftwareStrip(LEDStrip[] strips, int startIndex, boolean reversed) {
            this(strips, startIndex, LEDStrip.getLength(strips), reversed);
        }
        public SoftwareStrip(LEDStrip[] strips, int startIndex, int endIndex) {
            this(strips, startIndex, endIndex, false);
        }
        public SoftwareStrip(LEDStrip[] strips, int startIndex, int endIndex, boolean reversed) {
            this.strips = strips;
            this.startIndex = startIndex;
            this.endIndex = endIndex;
            this.reversed = reversed;
        }

        @Override
        public int getLength() {
            return endIndex - startIndex;
        }

        @Override
        public void setLED(int ledIndex, RawLEDColor color) {
            ledIndex = getLED(ledIndex);
            int accumLength = 0;
            for(LEDStrip strip : strips) {
                int stripLength = strip.getLength();
                accumLength += stripLength;
                if(accumLength >= ledIndex) {
                    accumLength -= stripLength;
                    ledIndex -= accumLength;
                    strip.setLED(ledIndex, color);
                    break;
                }
            }
        }

        @Override
        public SoftwareStrip reverse() {
            return new SoftwareStrip(strips, getLength() - endIndex, getLength() - startIndex, !reversed);
        }
        @Override
        public SoftwareStrip substrip(int startIndex, int endIndex) {
            return new SoftwareStrip(strips, clampToBounds(startIndex + this.startIndex), clampToBounds(endIndex + this.startIndex));
        }

        private int getLED(int ledIndex) {
            return getLED(ledIndex, reversed);
        }
        private int getLED(int ledIndex, boolean reversed) {
            ledIndex += startIndex;
            if(reversed) {
                ledIndex = getLength() - ledIndex - 1;
            }
            return ledIndex;
        }

        private int clampToBounds(int input) {
            return MathUtil.clamp(input, this.startIndex, this.endIndex);
        }
    }
}
