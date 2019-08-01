using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Waypoint {
    //waypoint class for waypoint objects of a path
    public Vector2 position;
    //0 = left
    //1 = centre
    //2 = right
    public float angleToNextWaypt;

    public Waypoint(Vector2 _pos)
    {
        position = _pos;
    }

    public void SetAngleToNext(float _angle)
    {
        angleToNextWaypt = _angle;
    }

    public void SetPosition(Vector2 _pos)
    {
        position = _pos;
    }

}
