package org.firstinspires.ftc.teamcode.RoverRuckus;

import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;

public class Fold extends Targetable< Fold.PosEnum >   //ManualTargeting< Fold.PosEnum >
{
    private static final int   FOLDED                     = 0;
    private static final int   VERTICAL                   = 690;
    private static final int   FLOOR                      = 2700;

    protected RevTouchSensor m_limitSw = null;

    public enum PosEnum
    {
        None,
        Folded,
        Vertical,
        Floor
    }
    // //////////////////////////////////////////////////////////////////////
    // override to implement limit switches
    // //////////////////////////////////////////////////////////////////////

    public Fold( DcMotor motor, RevTouchSensor limitSwitch )
    {
        super( motor );

        motor.setZeroPowerBehavior( DcMotor.ZeroPowerBehavior.BRAKE );

        m_limitSw = limitSwitch;

        m_eTargetPos = PosEnum.None;
    }

    @Override
    public boolean GetLimitBottom() { return m_limitSw.isPressed(); }
/*
    @Override public int    DeadZone() { return 30; }

    @Override public double ScalePower( double dPower, int nAbsAmtToGo )
    {
        if (nAbsAmtToGo < 50)
            return dPower * .50;

        if (nAbsAmtToGo < 25)
            return dPower * .25;

        return dPower;
    }
*/
    // //////////////////////////////////////////////////////////////////////
    //
    // //////////////////////////////////////////////////////////////////////

    @Override protected Fold.PosEnum GetNotTargetingValue() { return PosEnum.None;  }
    @Override protected Fold.PosEnum GetTopTarget()         { return PosEnum.Floor; }

    // //////////////////////////////////////////////////////////////////////
    //
    // //////////////////////////////////////////////////////////////////////

    @Override
    protected int GetEncoderTargetValue( Fold.PosEnum eTarget)
    {
        switch ( eTarget)
        {
            case Folded:    return FOLDED;
            case Vertical:  return VERTICAL;
            case Floor:     return FLOOR;
        }

        return 0;
    }
}
