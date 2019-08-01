using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Grid : MonoBehaviour {

    //wall are used to place nodes into closed set
    public LayerMask Walls;
    //size of the grid in world coordinates
    public Vector2 worldSize;
    //half of the width of a node
    public float nodeHalfWidth;
    //grid is built from node objects
    Node[,] grid;
    //full width of a node
    float nodeWidth;
    //x and y size of a grid based on node amount (not world coordinates)
    int gridSizeX, gridSizeY;


    private void Awake()
    {
        nodeWidth = nodeHalfWidth * 2;  //initialize node width
        gridSizeX = Mathf.RoundToInt(worldSize.x / nodeWidth);  //get amount of nodes in x
        gridSizeY = Mathf.RoundToInt(worldSize.y / nodeWidth);  //get amount of nodes in y
        CreateGrid();   //create the grid of nodes for A*
    }

    //get maximum size (or area) of the grid based on node amount
    public int GetMaxSize
    {
        get
        {
            return gridSizeX * gridSizeY;
        }
    }

    //create grid of nodes for A* consisting of driveable (0), semi-driveable (or close to a wall) (1), and undriveable(2) nodes
    void CreateGrid()
    {
        //starting from the bottom left corner of the grid...
        Vector2 worldBottomLeft;
        grid = new Node[gridSizeX, gridSizeY];
        worldBottomLeft.x = transform.position.x - worldSize.x / 2;
        worldBottomLeft.y = transform.position.y - worldSize.y / 2;

        //we need a 2D vector called boxsize to use built in OverlapBox physics function to find if current grid node intersects with a wall
        Vector2 boxSize;
        boxSize.x = nodeWidth;
        boxSize.y = nodeWidth;

        //iterate through all nodes in the grid
        for (int x = 0; x < gridSizeX; x++)
        {
            for (int y = 0; y < gridSizeY; y++)
            {
                //calculate the position of a node in world coordinates
                Vector2 worldNodeCoord = worldBottomLeft + Vector2.right * (x * nodeWidth + nodeHalfWidth) + Vector2.up * (y * nodeWidth + nodeHalfWidth);
                int wall = 0;
                if (Physics2D.OverlapBox(worldNodeCoord, boxSize, 0.0f, Walls)) //if node overlaps with a wall then it is undriveable 
                    wall = 2;
                else
                    wall = GetNeighbours(worldBottomLeft, boxSize, x, y);   //otherwise check whether the node is driveable or semi-driveable
                
                grid[x, y] = new Node(wall, worldNodeCoord, x, y);  //add node to the grid with new parameters
            }
        }
    }

    //gets neighboroughs of the current node to find whether it is driveable or semi-driveable
    public int GetNeighbours(Vector2 worldBottomLeft, Vector2 boxSize, int _x, int _y)
    {
        //initialize node to driveable
        int driveable = 0;

        //check whether the node is semi-driveable
        for (int x = _x - 1; x <= _x + 1; x++)
        {
            for (int y = _y - 1; y <= _y + 1; y++)
            {
                //find the location of the neighborough
                Vector2 worldLocation = worldBottomLeft + Vector2.right * (x * nodeWidth + nodeHalfWidth) + Vector2.up * (y * nodeWidth + nodeHalfWidth);
                if (Physics2D.OverlapBox(worldLocation, boxSize, 0.0f, Walls))  //if neighborough is undriveable then this node is semi-driveable
                {
                    driveable = 1;
                    break;
                }
            }
            if (driveable == 1)
                break;

        }
        return driveable;
    }

    //finds whether the car should drive through semi-driveable node
    public bool CalculateThinPath(Node _currentNode, Node _neighbourNode)
    {
        //find the direction to the neighborough node from current node
        int directionX = _neighbourNode.gridX - _currentNode.gridX;
        int directionY = _neighbourNode.gridY - _currentNode.gridY;
        //select next node after the neighborough in the same direction for checking.
        int checkX = _neighbourNode.gridX + directionX;
        int checkY = _neighbourNode.gridY + directionY;
        //initialize passed checks value
        int passedChecks = 0;
        
        //check whether next node after is not undriveable, if it's not then increment passed checks
        if (checkX >= 0 && checkX < gridSizeX && checkY >= 0 && checkY < gridSizeY)
        {
            passedChecks = (grid[checkX, checkY].driveable < 2) ? passedChecks + 1 : passedChecks;

        }
        //select node to the side of neighborough node for checking
        checkX = _neighbourNode.gridX + directionY;
        checkY = _neighbourNode.gridY - directionX;
        if (checkX >= 0 && checkX < gridSizeX && checkY >= 0 && checkY < gridSizeY)
        {
            passedChecks = (grid[checkX, checkY].driveable != 1) ? passedChecks : passedChecks + 1; //if node to the side is semi-driveable then increment passed checks
            passedChecks = (grid[checkX, checkY].driveable == 0) ? passedChecks - 1: passedChecks;  //if node to the side is driveable then decrement the passed checks

        }
        //select node to the other side of neighborough node for checking and perform same checks as previosuly
        checkX = _neighbourNode.gridX - directionY;
        checkY = _neighbourNode.gridY + directionX;
        if (checkX >= 0 && checkX < gridSizeX && checkY >= 0 && checkY < gridSizeY)
        {
            passedChecks = (grid[checkX, checkY].driveable  != 1) ? passedChecks : passedChecks + 1;
            passedChecks = (grid[checkX, checkY].driveable == 0) ? passedChecks - 1 : passedChecks;

        }
        //if the node passed more than one check then the node is a thin passage with no alternative routes around it and thus it can be traversed
        if (passedChecks > 1)
            return true;
        return false;
    }

    //get neighborous of the current node, return a list of neighboroughs
    public List<Node> GetNeighbours(Node node)
    {
        List<Node> neighbours = new List<Node>();

        for (int x = -1; x <= 1; x++)
        {
            for (int y = -1; y <= 1; y++)
            {
                if (x == 0 && y == 0)
                    continue;


                int checkX = node.gridX + x;
                int checkY = node.gridY + y;

                if (checkX >= 0 && checkX < gridSizeX && checkY >= 0 && checkY < gridSizeY)
                {
                    neighbours.Add(grid[checkX, checkY]);
                }
            }
            
        }

        return neighbours;
    }

    //get node x and y ID based on a point in world coordinates
    public Node NodeFromWorldCoord(Vector3 worldPosition)
    {
        int x = Mathf.RoundToInt((worldPosition.x + (worldSize.x / 2) - transform.position.x - nodeHalfWidth) / nodeWidth);
        int y = Mathf.RoundToInt((worldPosition.y + (worldSize.y / 2) - transform.position.y - nodeHalfWidth) / nodeWidth);

        x = Mathf.Clamp(x, 0, gridSizeX - 1);
        y = Mathf.Clamp(y, 0, gridSizeY - 1);

        return grid[x, y];
    }

    public Vector2[] path;


    private void OnDrawGizmos()
    {
         Gizmos.DrawWireCube(transform.position, new Vector2(worldSize.x, worldSize.y));

        //if (path != null)
        //{
        //    for (int i = 1; i < path.Length; i++)
        //    {
        //        Gizmos.color = Color.black;
        //        Gizmos.DrawCube(path[i], Vector3.one * (nodeWidth - 0.01f));
        //        Gizmos.DrawLine(path[i - 1], path[i]);
        //    }
        //}

        if (grid != null)
        {
            //  Node playerNode = NodeFromWorldPoint(player.position);
            foreach (Node n in grid)
            {
                switch (n.driveable)
                {
                    case 0: Gizmos.color = Color.white; break;
                    case 1: Gizmos.color = Color.cyan; break;
                    case 2: Gizmos.color = Color.red; break;
                }
                if(n.driveable != 0)
                {
                    Gizmos.DrawCube(n.worldPosition, Vector2.one * (nodeWidth - 0.03f));
                }
                //Gizmos.color = (n.driveable < 2) ? Color.white : Color.red;

                //if (playerNode.worldPosition == n.worldPosition)
                //{
                //    Gizmos.color = Color.cyan;
                //}


            }
        }
    }


}
