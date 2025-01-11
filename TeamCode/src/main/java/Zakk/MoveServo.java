package Zakk;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "TeleOpMain", group = "Testing")
public class MoveServo extends OpMode {

    /*
     * Declare Hardware
     */

    Servo Claw;

    @Override
    public void init() {

        Claw = hardwareMap.get(Servo.class, "Claw");
        Claw.setPosition(0);

    }

    @Override
    public void loop() {

        /*
         * Gamepad Controls
         */

        // Gamepad 1
        double oneLeftStickYPower = -gamepad1.left_stick_y;
        double oneLeftStickXPower = gamepad1.left_stick_x;
        double oneRightStickXPower = gamepad1.right_stick_x;

        double Position = Claw.getPosition();
        Claw.setPosition(Position + oneRightStickXPower);
        telemetry.addData("Claw Position",Claw.getPosition());

    }

}