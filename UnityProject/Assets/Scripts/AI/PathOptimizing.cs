using System.Collections;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using UnityEngine;

public class PathOptimizing : MonoBehaviour {

    public GameObject prototypeCar;   //prototype vehicle that traverses through waypoints and optimizes upon start
    List<Waypoint> waypoint;
    Checker[] checkers; //checkers are two points at both front edges of the car used for checking if there is a wall between car and waypoint
    Grid grid;

    private void Start()
    {
        //initialize all
        grid = GetComponent<Grid>();
        checkers = prototypeCar.GetComponentsInChildren<Checker>();
        waypoint = new List<Waypoint>();

    }

    //optimizes path using car's field of view, deleting unnecessary waypoints and moving other to a better position
    public List<Waypoint> OptPath(List<Vector2> _waypoints, Vector2 _start)
    {
        Stopwatch sws = new Stopwatch();
        sws.Start();
        
        List<Vector2> carPath = _waypoints;  //create local list of waypoints
        float lowerAngle = -60; //initialize lower and upper angle range used for checking for waypoints in front of a car
        float upperAngle = 60;

        //initialize variables
        float[] distances = new float[2];  
        Vector3 angleToWaypoint = new Vector3(0.0f, 0.0f, 0.0f);
        int j = 0;
        int startIter = 0;  //start iter variable used by algorithm to know at which waypoint it should be at currently
        prototypeCar.transform.position = _start;
        Vector2 position = prototypeCar.transform.position;
        
        for (int i = 0; i < carPath.Count; i++)
        {
            position = prototypeCar.transform.position;
            Quaternion rotation = prototypeCar.transform.rotation;
            j = startIter;  //we do not want to include waypoints behind the prototypeCar in our search 

            //get waypoints that will be checked by the field of view algorithm
            while (j < carPath.Count - 1)
            {
                j++;
                //get an angle from the car to the node, if it falls within angle range then iterate further through waypoints incrementing j, otherwise break loop
                angleToWaypoint.z = GetAngle(position, carPath[j], prototypeCar.transform.rotation);
                if (angleToWaypoint.z > upperAngle || angleToWaypoint.z < lowerAngle)
                    break;
            }

            //check whether the prototypeCar (prototype car) can see the goal (last) waypoint
            //get angle between car and last waypoint
            angleToWaypoint.z = GetAngle(position, carPath[carPath.Count - 1], prototypeCar.transform.rotation);
            //rotate towards last waypoint
            prototypeCar.transform.Rotate(angleToWaypoint);
            //get distance between prototypeCar and last waypoint
            float pointDist = Vector2.Distance(position, carPath[carPath.Count - 1]) - 0.5f;
            //find whether there is a wall before the waypoint
            distances = Raycasts(pointDist);
            //if there is no wall between prototypeCar car and last waypoint
            if (pointDist < distances[0] && pointDist < distances[1])
            {
                //delete all waypoints between car and last waypoint as they are not needed
                carPath.RemoveRange(startIter, carPath.Count - (startIter + 1));
                //set previous waypoint "angle to next" variable value to an angle between previous waypoint and target waypoint
                if(i > 0)
                    waypoint[i - 1].SetAngleToNext(GetAngle(position, carPath[i], rotation));
                //move prototypeCar to the target node
                prototypeCar.transform.position = carPath[startIter];
                //add the goal waypoint to the list of waypoints
                waypoint.Add(new Waypoint(prototypeCar.transform.position));
                if(i > 0)
                    waypoint[i].SetAngleToNext(waypoint[i - 1].angleToNextWaypt);
                sws.Stop();
                print("Optimizer: " + sws.ElapsedMilliseconds + " ms");
                //return list of waypoints
                GameObject.Destroy(prototypeCar); 
                return waypoint;
            }
            else  //if there is a wall between goal and prototypeCar then
            {



                //calculate distance between car and furthest away waypoint that fell within angle range
                pointDist = Vector2.Distance(position, carPath[j]) - 0.5f;
                //rotate towards the waypoint
                angleToWaypoint.z = GetAngle(position, carPath[j], prototypeCar.transform.rotation);
                prototypeCar.transform.Rotate(angleToWaypoint);
                //check whether there is wall before the waypoint
                distances = Raycasts(pointDist);
                //while distance to the waypoint is greater than distance to the wall in front of the prototypeCar check waypoints closer to prototypeCar
                while ((pointDist > distances[0] || pointDist > distances[1]) && j > 0)
                {
                    j--;    //decrement j to check the closer waypoint
                    //rotate towards the waypoint
                    angleToWaypoint.z = GetAngle(position, carPath[j], prototypeCar.transform.rotation);
                    prototypeCar.transform.Rotate(angleToWaypoint);

                    //get distance to the waypoint and distances to the wall in front of the prototypeCar
                    pointDist = Vector2.Distance(position, carPath[j]) - 0.5f;
                    distances = Raycasts(pointDist);
                }
                //if starting iteration is lower than furthest away waypoint without the wall before it (j), remove all waypoints between prototypeCar and j
                if (startIter < j)
                    carPath.RemoveRange(startIter, j - startIter);
                //if current iteration is greater than 1 then 
                if (i > 0)
                {
                    //set previous waypoint "angle to next" to an angle between current and previous waypoint
                    waypoint[i - 1].SetAngleToNext(GetAngle(position, carPath[i], rotation));
                    //rotate prototypeCar so that it has the same heading as it had at previous waypoint
                    prototypeCar.transform.Rotate(Vector3.forward, -waypoint[i - 1].angleToNextWaypt);
                    //move prototypeCar to the previous waypoint
                    prototypeCar.transform.position = waypoint[i - 1].position;
                    //change the position of previous waypoint furthest back as possible to optimize turning in corners
                    if (i > 1)
                        waypoint[i - 1].SetPosition(CheckHowFarBackCanTurn(carPath[i], carPath[i - 2]));
                    else
                        waypoint[i - 1].SetPosition(CheckHowFarBackCanTurn(carPath[i], _start));
                    //update previous waypoint angle to next
                    waypoint[i - 1].SetAngleToNext(GetAngle(prototypeCar.transform.position, carPath[i], rotation));

                }
                //move the prototypeCar to the current starting waypoint
                prototypeCar.transform.position = carPath[startIter];
                //increment starting iteration
                startIter++;
                //add current position of the prototypeCar (waypoint position) to the list
                waypoint.Add(new Waypoint(prototypeCar.transform.position));

            }

        }

        sws.Stop();
        carPath.RemoveRange(startIter, carPath.Count - (startIter + 1));
        print("Optimizer: " + sws.ElapsedMilliseconds + " ms");
        GameObject.Destroy(prototypeCar);
        return waypoint;
    }

    //method used for moving waypoints as far back as possible at corners so that cornering of the car is optimized
    Vector2 CheckHowFarBackCanTurn(Vector2 _nextNodeLoc, Vector2 _previousNodeLoc)
    {
        //initialize direction, position, angle and distances variables
        Vector3 direction = new Vector3(0, 1, 0);
        Quaternion initialRotation = prototypeCar.transform.rotation;
        direction = initialRotation * direction;
        Vector3 position = prototypeCar.transform.position;
        Vector3 angleToWaypoint = new Vector3(0.0f, 0.0f, 0.0f);
        float pointDist = 0f;
        Node checkNode = grid.NodeFromWorldCoord(prototypeCar.transform.position);
        float[] distances = Raycasts(pointDist);

        //calculating the angle between three waypoints
        float lm = Vector2.Distance(_nextNodeLoc, prototypeCar.transform.position);
        float lr = Vector2.Distance(_previousNodeLoc, _nextNodeLoc);
        float rm = Vector2.Distance(_previousNodeLoc, prototypeCar.transform.position);

        float lmSquared = lm * lm;
        float lrSquared = lr * lr;
        float rmSquared = rm * rm;

        float ratio = (rmSquared + lmSquared - lrSquared) / (2 * lm * rm);

        double initialDegree = Mathf.Acos(ratio) * (180 / Math.PI);
        double degree = Mathf.Acos(ratio) * (180 / Math.PI);

        //while prototypeCar can still see the next waypoint, move it 0.5 units back and check whether it can still see the next waypoint
        do
        {
            position = prototypeCar.transform.position;
            prototypeCar.transform.position = position - direction * 0.5f;    //move prototypeCar back 0.5f units based on its direction
            //rotate prototypeCar to the next waypoint and check whether it can still see it
            angleToWaypoint.z = GetAngle(prototypeCar.transform.position, _nextNodeLoc, prototypeCar.transform.rotation);
            prototypeCar.transform.Rotate(angleToWaypoint);
            pointDist = Vector2.Distance(prototypeCar.transform.position, _nextNodeLoc) - 0.5f;
            distances = Raycasts(pointDist);
            //make sure prototypeCar did not go so far back that it touched a wall
            checkNode = grid.NodeFromWorldCoord(prototypeCar.transform.position);

            lm = Vector2.Distance(_nextNodeLoc, prototypeCar.transform.position);
            lr = Vector2.Distance(_previousNodeLoc, _nextNodeLoc);
            rm = Vector2.Distance(_previousNodeLoc, prototypeCar.transform.position);

            lmSquared = lm * lm;
            lrSquared = lr * lr;
            rmSquared = rm * rm;

            ratio = (rmSquared + lmSquared - lrSquared) / (2 * lm * rm);

            degree = Mathf.Acos(ratio) * (180 / Math.PI);

        } while ((pointDist < distances[0] && pointDist < distances[1]) && checkNode.driveable == 0 && degree >= initialDegree);

        return prototypeCar.transform.position = position;    //return prototypeCar position as the new waypoint position
    }

    //Not a real raycast, does not use physics, moves checkers forward until they reach an undriveable node, then returns their distance from the prototypeCar
    float[] Raycasts(float _checkPointDist)
    {
        float[] distances = new float[2];
        float nodeJumpDist = grid.nodeHalfWidth / 2;    //"jump distance" is the distance checkers will traverse at each iteration

        for (int i = 0; i < checkers.Length; i++)
        {
            //get node at checkers location and initialize other variables
            Node checkNode = grid.NodeFromWorldCoord(checkers[i].transform.position);
            Vector3 position = checkers[i].transform.position;
            Vector3 direction = new Vector3(0, 1, 0);
            Quaternion Rotation = checkers[i].transform.rotation;
            direction = Rotation * direction;
            bool collisionFound = false;
            int j = 0;
            while (!(collisionFound))   //until undriveable node is reached, move forward and update distances
            {
                j++;
                position = checkers[i].transform.position;
                checkers[i].transform.position = position + direction * nodeJumpDist;
                checkNode = grid.NodeFromWorldCoord(checkers[i].transform.position);
                //if node at checkers position is not driveable or checkers are farther away than the waypoint, save checkers distance and move checker distance back to its initial pos
                if (checkNode.driveable == 2 || j > _checkPointDist / nodeJumpDist)
                {
                    distances[i] = j * nodeJumpDist;
                    checkers[i].transform.position = position - direction * nodeJumpDist * (j - 1);
                    collisionFound = true;
                }
            }
        }
        return distances;
    }

    //method used to get angle between two points based on rotation and location of starting point and location of target point
    float GetAngle(Vector3 _startPos, Vector3 _targetPos, Quaternion _rotation)
    {
        Vector3 direction = new Vector3(0, 1, 0);
        direction = _rotation * direction;

        Vector3 target = new Vector3(_targetPos.x, _targetPos.y, 0.0f);
        Vector3 targetDir = _targetPos - _startPos;

        float angle = Vector3.Angle(direction, targetDir);
        Vector3 cross = Vector3.Cross(direction, targetDir);

        if (cross.z < 0)
            angle = -angle;


        return angle;
    }
}
