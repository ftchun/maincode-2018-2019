package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name = "Gyro Test", group = "Linear")
public class ColorTest extends LinearOpMode {

    private BNO055IMU imu;

    double globalAngle;

    Orientation lastAngles = new Orientation();
	
    @Override
    public void runOpMode() throws InterruptedException {


        imu = hardwareMap.get(BNO055IMU.class, "imu");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        imu.initialize(parameters);



        waitForStart();

        while(opModeIsActive()) {
            getAngle();

            telemetry.addData("angle", globalAngle);

            telemetry.update();
        }
    }

    private void getAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = angles.firstAngle;


        //double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        //if (deltaAngle < -180)
        //    deltaAngle += 360;
        //else if (deltaAngle > 180)
        //    deltaAngle -= 360;

        //globalAngle += deltaAngle;

        //lastAngles = angles;

        //return globalAngle;
    }
}
