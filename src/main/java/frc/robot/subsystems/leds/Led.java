package frc.robot.subsystems.leds;

import com.ctre.phoenix6.configs.LEDConfigs;
import com.ctre.phoenix6.controls.RainbowAnimation;
import com.ctre.phoenix6.controls.SingleFadeAnimation;
import com.ctre.phoenix6.controls.StrobeAnimation;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StripTypeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.leds.LedConstants.LEDStates;
import frc.robot.util.PhoenixUtil;

public class Led extends SubsystemBase{
    CANdle leds;
    LEDConfigs configs;

    public Led() {
        leds = new CANdle(LedConstants.ledsID);
        configs = new LEDConfigs();
        configs.StripType = StripTypeValue.GRB;
        PhoenixUtil.tryUntilOk(5, () -> leds.getConfigurator().apply(configs));
    }

    public void setLEDS(LEDStates state) {
        switch (state) {
            case Default:
                leds.setControl(
                    new StrobeAnimation(0, LedConstants.LedEndIndex)
                    .withColor(new RGBWColor(0, 255, 255))
                    .withFrameRate(250));
                break;
            case Intaking:
                leds.setControl(
                    new StrobeAnimation(0, LedConstants.LedEndIndex)
                    .withColor(new RGBWColor(255, 0, 255))
                    .withFrameRate(250));
                // leds.animate(new StrobeAnimation(255, 0, 255, 0, 0.5, LedCount));
                break;
            case NotInScoringPosition:
                leds.setControl(
                    new StrobeAnimation(0, LedConstants.LedEndIndex)
                    .withColor(new RGBWColor(0, 255, 255))
                    .withFrameRate(400));
                // leds.animate(new StrobeAnimation(0, 255, 255, 0, 0.8, LedCount));
                break;
            case InScoringPosition:
                leds.setControl(
                    new StrobeAnimation(0, LedConstants.LedEndIndex)
                    .withColor(new RGBWColor(0, 255, 0)));
                // leds.animate(new StrobeAnimation(0, 255, 0));
                break;
            case DisabledRed:
                leds.setControl(
                    new SingleFadeAnimation(0, LedConstants.LedEndIndex)
                    .withColor(new RGBWColor(255, 0, 0))
                    .withFrameRate(400));
                // leds.animate(new SingleFadeAnimation(255, 0, 0, 0, 0.7, LedCount));
                break;
            case DisabledBlue:
                leds.setControl(
                    new SingleFadeAnimation(0, LedConstants.LedEndIndex)
                    .withColor(new RGBWColor(0, 0, 255))
                    .withFrameRate(400));
                // leds.animate(new SingleFadeAnimation(0, 0, 255, 0, 0.7, LedCount));
                break;
            case Endgame:
                leds.setControl(
                    new RainbowAnimation(0, LedConstants.LedEndIndex)
                    .withFrameRate(1000));
                // leds.animate(new RainbowAnimation(1, 1, LedCount));
                break;
            default:
                break;
        }
    }
    public Command setLedCommand(LEDStates LedState){
        return Commands.runOnce(()-> setLEDS(LedState));
    }
}
