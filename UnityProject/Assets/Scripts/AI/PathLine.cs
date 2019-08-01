using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public struct PathLine
{
    const float vertical = 100000f;

    float slope;
    float y_intercept;
    Vector2 pointOnLine1;
    Vector2 pointOnLine2;

    float slopePerpendicular;

    bool approachSide;

    public PathLine(Vector2 _pointOnLine, Vector2 _pointPerpendicular)
    {
        float dx = _pointOnLine.x - _pointPerpendicular.x;
        float dy = _pointOnLine.y - _pointPerpendicular.y;

        if(dx == 0) //if the run is equal to 0 then slope perpendicular should pe vertical and checkpoint line horizontal
        {
            slopePerpendicular = vertical;
        }
        else  //otherwise the gradient is rise divided by run
        {
            slopePerpendicular = dy / dx;
        }
        if (slopePerpendicular == 0)    //if slope of the line is 0 then the checkpoint line should be vertical
        {
            slope = vertical;
        }
        else  //otherwise checkpoint line should be -1 / perpendicular slope
        {
            slope = -1 / slopePerpendicular;
        }


        y_intercept = _pointOnLine.y - slope * _pointOnLine.x;  // set the intercept or middle of the line to the waypoint location - direction
        //calculate line coords
        pointOnLine1 = _pointOnLine;
        pointOnLine2 = _pointOnLine + new Vector2(1, slope);

        approachSide = false;
        //find the side on which the perpendicular point is
        approachSide = GetSide(_pointPerpendicular);
    }

    //find whether the point is on correct side of the line
    bool GetSide(Vector2 _p)
    {
        return (_p.x - pointOnLine1.x) * (pointOnLine2.y - pointOnLine1.y) > (_p.y - pointOnLine1.y) * (pointOnLine2.x - pointOnLine1.x);
    }

    //if a point approached from the correct side (other than perpendicular side) then it passed it
    public bool CrossedLine(Vector2 _p)
    {
        return GetSide(_p) != approachSide;
    }

    public void DrawWithGizmos(float _length)
    {
        Vector2 lineDir = new Vector2(1, slope).normalized;
        Vector2 lineCentre = new Vector2(pointOnLine1.x, pointOnLine1.y);
        Gizmos.DrawLine(lineCentre - lineDir * _length / 2f, lineCentre + lineDir * _length / 2f);
    }
}
