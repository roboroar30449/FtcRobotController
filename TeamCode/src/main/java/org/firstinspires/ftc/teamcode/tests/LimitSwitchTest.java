package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;

@TeleOp(name = "Limit Switch Test", group = "Testing")
public class LimitSwitchTest extends OpMode {

    public AnalogInput limitSwitch0;
    public AnalogInput limitSwitch1;

    @Override
    public void init() {
        limitSwitch0 = hardwareMap.get(AnalogInput.class, "limitSwitch0");
        limitSwitch1 = hardwareMap.get(AnalogInput.class, "limitSwitch1");
        telemetry.addLine("Initialized (Analog Input)");
    }

    @Override
    public void loop() {
        double voltage0 = limitSwitch0.getVoltage();
        double voltage1 = limitSwitch1.getVoltage();
        boolean isPressed0 = voltage0 > 0.0009;
        boolean isPressed1 = voltage1 > 0.0009;

        telemetry.addData("Voltage0", voltage0);
        telemetry.addData("Voltage1", voltage1);
        telemetry.addData("Switch Status", isPressed0 ? "Pressed" : "Not Pressed");
        telemetry.addData("Switch Status", isPressed1 ? "Pressed" : "Not Pressed");
        telemetry.update();
    }
}
