using System.Collections;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using UnityEngine;

public class PathFinding : MonoBehaviour {

    public Transform start, end;    //start and goal position of the path

    List<Waypoint> waypoint;
    Grid grid;
    PathOptimizing optimizer;



    private void Start()
    {
        grid = GetComponent<Grid>();
        optimizer = GetComponent<PathOptimizing>();
        waypoint = new List<Waypoint>();

    }

    //starts finding the path when called
    public List<Waypoint> StartFinding(Vector3 _startPos)
    {
        //slightly modified A* search finds initial path
        List<Vector2> waypoints = FindPath(_startPos, end.position);
        //field of view algorithm optimizes the path by deleting and moving path waypoints
        List<Waypoint> way = optimizer.OptPath(waypoints, start.position);
        print("Amount of waypoints optimized: " + way.Count);
        return way;

    }


    List<Vector2> FindPath(Vector3 _startPos, Vector3 _targetPos)
    {
        Stopwatch sw = new Stopwatch();
        sw.Start();

        //new empty list of vector2 that will be filled with path waypoints
        List<Vector2> waypoints = new List<Vector2>();
        bool pathFound = false;

        //Get start and goal nodes
        Node start = grid.NodeFromWorldCoord(_startPos);
        Node target = grid.NodeFromWorldCoord(_targetPos);

        //check for a path only if start and end nodes are driveable
        if (start.driveable != 2 && target.driveable != 2)
        {
            //create an empty heap of nodes of grid size that will be used for searching the path
            Heap<Node> openSet = new Heap<Node>(grid.GetMaxSize);
            //create a closed hash set that consists of already checked nodes
            HashSet<Node> closedSet = new HashSet<Node>();
            openSet.Add(start); //add start node to open set
            //while there are still nodes in the open set
            while (openSet.Count > 0)
            {
                Node currentNode = openSet.RemoveFirst();   //remove first node (lowest f score) from the open set and set it as current

                closedSet.Add(currentNode); //add current node to the closed set

                if (currentNode == target)  //if current node equals the goal node then path is found
                {
                    sw.Stop();
                    print("Path found: " + sw.ElapsedMilliseconds + " ms");
                    pathFound = true;
                    break;
                }

                foreach (Node neighbour in grid.GetNeighbours(currentNode)) //for each neighbour of the current node
                {
                    if (neighbour.driveable == 2 || closedSet.Contains(neighbour))  //find whether neighbour is not driveable or alrady in the closed set
                    {
                        continue;
                    }

                    int movementCost = currentNode.gCost + GetDistance(currentNode, neighbour);   //calculate movement cost to neighbour node
                    //if movement cost to neighbour is lower than neighbour g cost or openset does not contain neighbour (gCost == 0) then...
                    if (movementCost < neighbour.gCost || !openSet.Contains(neighbour)) 
                    {
                        bool thinPath = false;
                        if (neighbour.driveable == 1)   //if neighbour being checked is semi-driveable then check whether it can be a part of thin passage
                        {
                            thinPath = grid.CalculateThinPath(currentNode, neighbour);
                        }
                        if (neighbour.driveable == 0 || thinPath == true) //if neighbour is driveable or a part of thin passage way then add and sort it
                        {
                            neighbour.gCost = movementCost; //current neighbour g cost equals previously calculated g cost
                            neighbour.hCost = GetDistance(neighbour, target);   //h cost equals calculated distance (node amount) between goal and neighbour
                            neighbour.parent = currentNode; //parent of neighbour equals current node

                            if (!openSet.Contains(neighbour))   //if neighbour is not in the open set then add it to open set
                                openSet.Add(neighbour);
                            else  //if it is in open set then sort heap nodes
                                openSet.UpdateItem(neighbour);  //sort up items in the heap based on their f score, so that the item with lowest f score will be checked first
                        }
                        else
                            continue;

                    }
                }
            }
        }
        if (pathFound)  //retrace waypoints back to the start with the use of parent nodes
        {
            waypoints = Retrace(start, target);         
        }

        return waypoints;   //return simplified path
    }

    List<Vector2> Retrace(Node _startNode, Node _endNode)
    {
        //retrace the path from finish to start with the use of parents
        List<Node> path = new List<Node>();
        Node current = _endNode;

        while (current != _startNode)
        {
            path.Add(current);
            current = current.parent;
        }
        print("Amount of normal waypoints: " + path.Count);
        List<Vector2> waypoints = SimplePath(path); //simplifies path by deleting any waypoints that are going in the same diretion as previous waypoint
        print("Amount of simplified waypoints: " + waypoints.Count);
        grid.path = waypoints.ToArray();
        waypoints.Reverse();    //reverse waypoints as the path was retraced from finish to start

        return waypoints;
    }

    List<Vector2> SimplePath(List<Node> _path)
    {
        //simplifies path by deleting any waypoints that are going in the same diretion as previous waypoint
        List<Vector2> waypoints = new List<Vector2>();
        Vector2 oldDirection = Vector2.zero;

        for (int i = 1; i < _path.Count; i++)
        {
            //get the direction of the current node
            Vector2 direction = new Vector2(_path[i - 1].gridX - _path[i].gridX, _path[i - 1].gridY - _path[i].gridY);
            //if current waypoint direction does not equal previous waypoint direction then add the waypoint to the list
            if (direction != oldDirection)
            {
                waypoints.Add(_path[i - 1].worldPosition);
            }
            oldDirection = direction;

        }
        return waypoints;   //return updated waypoints
    }

    //get distance between nodes
    int GetDistance(Node _nodeA, Node _nodeB)
    {
        int dstX = Mathf.Abs(_nodeA.gridX - _nodeB.gridX);
        int dstY = Mathf.Abs(_nodeA.gridY - _nodeB.gridY);
        if(dstX > dstY)
        {
            return 14 * dstY + 10 * (dstX - dstY);
        }
        return 14 * dstX + 10 * (dstY - dstX);
    }

    public List<Waypoint> GetWaypointObjects()
    {
        return waypoint;
    }

}
