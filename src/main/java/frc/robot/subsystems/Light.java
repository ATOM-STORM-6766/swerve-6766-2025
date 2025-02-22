package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Light extends SubsystemBase {
    private static Light light;

    public static Light getInstance() {
        if (light == null) {
            light = new Light();
        }
        return light;
    }

    private AddressableLED m_led = new AddressableLED(0);
    private AddressableLEDBuffer m_blackLedBuffer;
    private AddressableLEDBuffer m_LedBuffer;
    private AddressableLEDBuffer m_buffer;
    private Notifier m_notifier;

    private Light() {
        m_led.setLength(60);
        m_blackLedBuffer = new AddressableLEDBuffer(60);
        m_LedBuffer = new AddressableLEDBuffer(60);
        m_led.start();
        m_notifier = new Notifier(() -> {
        });
        setStart();
        m_notifier.setName("FlashingLight");
    }

    public void setStart() {
        m_LedBuffer.forEach((index, r, g, b) -> {
            m_LedBuffer.setLED(index,
                    Constants.alliance == Alliance.Red ? Color.kRed : Color.kBlue// Color.kWhite//
            );
        });
        m_led.setData(m_LedBuffer);
    }

    public void setRainbow() {
        m_LedBuffer.forEach((i, r, g, b) -> {
            m_LedBuffer.setHSV(i, (int) (i * 180.0 / m_LedBuffer.getLength()), 255, 128);
        });
        m_notifier.setCallback(() -> {
            m_LedBuffer.forEach((i, r, g, b) -> {
                m_LedBuffer.setLED(i, m_LedBuffer.getLED((i + 1) % m_LedBuffer.getLength()));
            });
            m_led.setData(m_LedBuffer);
        });
        m_notifier.startPeriodic(0.0083);// 0.04166);
    }

    public void setBlack() {
        m_led.setData(m_blackLedBuffer);
    }

    public void autoMoving() {
        m_LedBuffer.forEach((index, r, g, b) -> {
            m_LedBuffer.setLED(index, Color.kGreen);
        });
        m_buffer = m_LedBuffer;
        m_notifier.setCallback(() -> {
            if (m_buffer == m_LedBuffer)
                m_buffer = m_blackLedBuffer;
            else
                m_buffer = m_LedBuffer;
            m_led.setData(m_buffer);
        });
        m_notifier.startPeriodic(0.3);
    }

    public void autoMovingStop() {
        m_notifier.stop();
        m_led.setData(m_LedBuffer);
    }

    public void moveElevator() {
        m_LedBuffer.forEach((index, r, g, b) -> {
            m_LedBuffer.setLED(index, new Color8Bit("#ff5c00"));
        });
        m_buffer = m_LedBuffer;
        m_notifier.setCallback(() -> {
            if (m_buffer == m_LedBuffer)
                m_buffer = m_blackLedBuffer;
            else
                m_buffer = m_LedBuffer;
            m_led.setData(m_buffer);
        });
        m_notifier.startPeriodic(0.3);
    }

    public void moveElevatorStop() {
        m_notifier.stop();
        m_led.setData(m_LedBuffer);
    }

    public void emptyMouth() {
        m_LedBuffer.forEach((index, r, g, b) -> {
            m_LedBuffer.setLED(index, Color.kBlueViolet);
        });
        m_buffer = m_LedBuffer;
        m_notifier.setCallback(() -> {
            if (m_buffer == m_LedBuffer)
                m_buffer = m_blackLedBuffer;
            else
                m_buffer = m_LedBuffer;
            m_led.setData(m_buffer);
        });
        m_notifier.startPeriodic(0.3);
    }

    public void fullMouth() {
        m_notifier.stop();
        m_led.setData(m_LedBuffer);
    }

    public void interrupted() {
        m_notifier.stop();
        m_led.setData(m_blackLedBuffer);
    }

}
