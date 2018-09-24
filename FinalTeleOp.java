package org.firstinspires.ftc.teamcode;

// Importing the information for
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp (name = "Final TeleOp", group = "Linear")
public class FinalTeleOp extends LinearOpMode {

    /*
        Here we declare the motor variables for driving. They are named "motor" and then the initials
        for which of the four they are. "F" stands for "front," "B" stands for "back," "R" stands for
        right, and "L" stands for "left." Therefore, "motorFL" is the front-left motor, "motorBR" is
        the back-right motor, etc.
    */

    private DcMotor motorFL;
    private DcMotor motorFR;
    private DcMotor motorBL;
    private DcMotor motorBR;

    /*
        Here we declare the motor variables for the peripheral motors: the arm and the claw, which is
        mounted on the arm. "motorArm" is the arm swinging motor, and "motorClaw" is the motor for the
        glyph-squeezing claw.
     */

    private DcMotor motorArm;
    //private DcMotor motorClaw;

    /*
       Here we are setting the thresholds for the arm and claw. This is so that the peripherals operator
       cannot accidentally swing the arm any lower or higher than we want them to, or squeeze the glyph too hard.
       The numbers are the minimum or maximum encoder position on the motors.
     */

    int armLowThreshold = 150;
    int armHighThreshold = 2100;

    /*
        This is the amount of encoder position the arm and claw motors move each time the loop goes
        through and the motors are triggered.
     */

    int movementDelta = 10;

    private Servo servoL;
    private Servo servoR;

    @Override
    public void runOpMode() throws InterruptedException {

        /*
            First we assign the motor variables to their respective hardware configurations.
            The configuration names in the phone are the same as they are in the code so that it is
            easy to remember.

            Then we set the direction for the left motors so that they match up with the right ones,
            so when we use a positive value they go forward.

            After that we set the modes for the motors with encoders on them. These include the arm
            motor, the claw motor, and the back right drive motor. The one on the drive motor is to
            measure the driving for autonomous. For each motor with an encoder, we reset them to zero,
            and for the arm motors we also set them to run to position mode.

            After this we declare two variables that serve as encoder positions. These go through a
            number of conditions later in the code for checking various threshold values and so forth.
         */

        motorFL = hardwareMap.dcMotor.get("motorFL");
        motorFR = hardwareMap.dcMotor.get("motorFR");
        motorBL = hardwareMap.dcMotor.get("motorBL");
        motorBR = hardwareMap.dcMotor.get("motorBR");
        motorArm = hardwareMap.dcMotor.get("motorArm");

        servoL = hardwareMap.servo.get("servoL");
        servoR = hardwareMap.servo.get("servoR");

        motorArm.setDirection(DcMotor.Direction.REVERSE);
        motorBL.setDirection(DcMotor.Direction.REVERSE);
        motorFL.setDirection(DcMotor.Direction.REVERSE);

        motorArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        int armPositionCount = 0;

        double servoPositionL = 0;
        double servoPositionR = 0.9;
        int speedConstant = 1;
        boolean prevX = false;
        boolean prevY = false;
        
        //rotating speed
        double servoDelta = 0.02;

        /*
            This line of code sets the arm to its base position on initialization.
         */

        gotoArmPosition(armLowThreshold, .5);

        Thread.sleep(50);

        servoL.setPosition(servoPositionL);
        servoR.setPosition(servoPositionR);

        waitForStart();

        while (opModeIsActive()) {

            /*
                This block of code allows the drive motors' power to be controlled by the left
                and right joysticks of Gamepad 1. The
             */
            
            if(prevY == false && gamepad1.y && speedConstant>0)
            {
                speedConstant -= 1;
            }
            if(prevX == false && gamepad1.x && speedConstant<2)
            {
                speedConstant += 1;
            }
            
            prevX = gamepad1.x;
            prevY = gamepad1.y;
            
            if(speedConstant == 0)
            {
                motorBL.setPower(-gamepad1.left_stick_y/2);
                motorFL.setPower(-gamepad1.left_stick_y/2);
                motorBR.setPower(-gamepad1.right_stick_y/2);
                motorFR.setPower(-gamepad1.right_stick_y/2);
            }
            
            if(speedConstant == 1)
            {
                motorBL.setPower(-gamepad1.left_stick_y/4);
                motorFL.setPower(-gamepad1.left_stick_y/4);
                motorBR.setPower(-gamepad1.right_stick_y/4);
                motorFR.setPower(-gamepad1.right_stick_y/4);
            }
            
            if(speedConstant == 2)
            {
                motorBL.setPower(-gamepad1.left_stick_y/8);
                motorFL.setPower(-gamepad1.left_stick_y/8);
                motorBR.setPower(-gamepad1.right_stick_y/8);
                motorFR.setPower(-gamepad1.right_stick_y/8);
            }

            if (gamepad2.dpad_up) {
                armPositionCount += movementDelta;
            } else if (gamepad2.dpad_down) {
                armPositionCount -= movementDelta;
            }

            if (armPositionCount < armLowThreshold) {
                armPositionCount = armLowThreshold;
            } else if (armPositionCount > armHighThreshold) {
                armPositionCount = armHighThreshold;
            }

            gotoArmPosition(armPositionCount, .5);



            if(gamepad2.left_trigger > 0){
                servoPositionR += servoDelta;
                servoPositionL -= servoDelta;
            }
            if(gamepad2.right_trigger > 0){
                servoPositionR -= servoDelta;
                servoPositionL += servoDelta;
            }
            servoPositionL = Range.clip(servoPositionL, 0, 0.8);
            servoPositionR = Range.clip(servoPositionR, 0.1, 0.9);
            servoR.setPosition(servoPositionR);
            servoL.setPosition(servoPositionL);

            telemetry.addData("servo L pos", servoL.getPosition());
            telemetry.addData("servo R pos", servoR.getPosition());



            int armPosition = motorArm.getCurrentPosition();
            int motorBRPosition = motorBR.getCurrentPosition();
            double armPower = motorArm.getPower();

            telemetry.addData("Arm Position: ", armPosition);
            telemetry.addData("Arm Power: ", armPower);
            telemetry.addData("motorBR Position: ", motorBRPosition);
            telemetry.addData("The Speed: ", speedConstant);
            telemetry.update();

            idle();
        }

    }

    private void gotoArmPosition (int position, double power) {
        motorArm.setTargetPosition(position);
        motorArm.setPower(power);
    }
}
