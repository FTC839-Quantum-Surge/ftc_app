package org.firstinspires.ftc.teamcode.RoverRuckus;

import com.qualcomm.robotcore.hardware.DcMotor;

public class Fold extends Targetable< Fold.PosEnum >
{
    private static final int   FOLDED                     = 0;
    private static final int   VERTICAL                   = 1000;
    private static final int   FLOOR                      = 2000;

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

    public Fold( DcMotor motor )
    {
        super( motor );

        m_eTargetPos = PosEnum.None;
    }

    // //////////////////////////////////////////////////////////////////////
    //
    // //////////////////////////////////////////////////////////////////////

    @Override protected Fold.PosEnum GetNotTargetingValue() { return PosEnum.None; }

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
