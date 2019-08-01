using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Node : IHeapItem<Node>{

    public int driveable;
    public Vector2 worldPosition;
    public int gridX;
    public int gridY;

    public int gCost;
    public int hCost;
    public Node parent;
    int hIndex;

    //set node values
    public Node(int _driveable, Vector2 _worldPos, int _gridX, int _gridY)
    {
        driveable = _driveable;
        worldPosition = _worldPos;
        gridX = _gridX;
        gridY = _gridY;

    }

    public int FCost    //get f score
    {
        get
        {
            return gCost + hCost;
        }
    }

    public int HIndex   //get and set heap index
    {
        get
        {
            return hIndex;
        }
        set
        {
            hIndex = value;
        }
    }

    public int CompareTo(Node _node)
    {
        int compare = FCost.CompareTo(_node.FCost);
        if (compare == 0)   //if both have the same F cost then compare H cost of the two
        {
            compare = hCost.CompareTo(_node.hCost);
        }
        return -compare;
    }
}
