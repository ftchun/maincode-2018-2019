package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "motor encode test", group = "Linear")
public class MotorEncoderTest extends LinearOpMode {

    private DcMotor motor;

    @Override
    public void runOpMode() {
        motor = hardwareMap.dcMotor.get("motor");

        double motorSpeed = 0;

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.right_bumper) {
                motorSpeed = .5;
            } else if (gamepad1.left_bumper) {
                motorSpeed = -.5;
            } else {
                motorSpeed = 0;
            }

            motor.setPower(motorSpeed);

            int position = motor.getCurrentPosition();

            telemetry.addData("position", position);

            telemetry.update();
        }


    }
}
