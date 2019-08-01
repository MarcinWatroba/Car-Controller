using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Path{

    public Vector2[] points;
    public PathLine[] turnBounds;
    public int finishLine;

    //create path out of perpendicular lines as checkpoints
    public Path(Vector2[] _waypoints, Vector2 _startPos)
    {
        points = _waypoints;
        turnBounds = new PathLine[points.Length];
        finishLine = turnBounds.Length - 1;
        Vector2 previousPoint = _startPos;
        for(int i = 0; i < points.Length; i++)
        {
            Vector2 currentPoint = points[i];
            Vector2 dirToCurrentPoint = (currentPoint - previousPoint).normalized;
            Vector2 boundaryPoint = currentPoint - dirToCurrentPoint; 
            turnBounds[i] = new PathLine(boundaryPoint, previousPoint - dirToCurrentPoint); //set checkpoint bounds for turning as perpendicular lines
            previousPoint = currentPoint;
        }

    }

    public void DrawWithGizmos()
    {
        Gizmos.color = Color.black;
        foreach(Vector2 p in points)
        {
            Gizmos.DrawCube(p, Vector2.one * 0.3f);
        }

        Gizmos.color = Color.gray;
        foreach(PathLine l in turnBounds)
        {
            l.DrawWithGizmos(10);
        }
    }

    
}
