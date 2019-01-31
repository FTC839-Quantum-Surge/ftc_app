package org.firstinspires.ftc.teamcode.RoverRuckus;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.AnalogInput;

public class Lift  extends Targetable< Lift.PosEnum >
{
    private static final int   LIFT_TOP                     = 19725;
    private static final int   LIFT_LATCH                   = 11250; //11000=not working  //11500   //12000;    //10000;

    private AnalogInput m_limitTop;
    private AnalogInput m_limitBottom;

    public enum PosEnum
    {
        None,
        Top,
        Hook,
        Bottom
    }
    // //////////////////////////////////////////////////////////////////////
    // override to implement limit switches
    // //////////////////////////////////////////////////////////////////////

    public Lift( DcMotor motor, AnalogInput limitTop, AnalogInput limitBottom )
    {
        super( motor );

        m_eTargetPos = PosEnum.None;

        m_limitBottom = limitBottom;
        m_limitTop    = limitTop;
    }

    // //////////////////////////////////////////////////////////////////////
    //
    // //////////////////////////////////////////////////////////////////////

    @Override protected Lift.PosEnum GetNotTargetingValue() { return PosEnum.None; }

    @Override public boolean  GetLimitTop   () { return m_limitTop   .getVoltage() > 0.5; }
    @Override public boolean  GetLimitBottom() { return m_limitBottom.getVoltage() > 0.5; }

    public double  GetLimitTopVal   () { return m_limitTop   .getVoltage(); }
    public double  GetLimitBottomVal() { return m_limitBottom.getVoltage(); }

    // //////////////////////////////////////////////////////////////////////
    //
    // //////////////////////////////////////////////////////////////////////

    @Override
    protected int GetEncoderTargetValue( Lift.PosEnum eTarget)
    {
        switch ( eTarget)
        {
            case Top:       return LIFT_TOP;
            case Hook:      return LIFT_LATCH;
            case Bottom:    return m_nHomeEncPos;
        }

        return 0;
    }
}
