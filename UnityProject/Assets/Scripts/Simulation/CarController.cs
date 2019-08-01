/// Author: Samuel Arzt
/// Date: March 2017

#region Includes
using UnityEngine;
using System.Collections.Generic;
using System.Diagnostics;
using System.Collections;
#endregion

/// <summary>
/// Class representing a controlling container for a 2D physical simulation
/// of a car with 5 front facing sensors, detecting the distance to obstacles.
/// </summary>
public class CarController : MonoBehaviour
{
    #region Members
    #region IDGenerator
    // Used for unique ID generation
    private static int idGenerator = 0;
    /// <summary>
    /// Returns the next unique id in the sequence.
    /// </summary>
    private static int NextID
    {
        get { return idGenerator++; }
    }
    #endregion

    // Maximum delay in seconds between the collection of two checkpoints until this car dies.
    private const float MAX_CHECKPOINT_DELAY = 500;

    public GameObject pathObject;
    PathFinding pathFind;
    Path linepath;

    private float initialRotation;
    Vector2[] waypoints;
    Vector2 nextGoal2Pos;
    Vector2 nextGoalPos;
    Vector2 goalPos;
    float angleToNextWpt;
    float angleToNMultipleWpts;
    float distBetweenWpt;
    
    bool updatedAngles;
    float stoppingDist;
    List<Waypoint> waypoint;
    int iterat;
    Stopwatch sw;


    /// <summary>
    /// Whether this car is controllable by user input (keyboard).
    /// </summary>
    public bool UseUserInput = false;

    /// <summary>
    /// The movement component of this car.
    /// </summary>
    public CarMovement Movement
    {
        get;
        private set;
    }

    /// <summary>
    /// The current inputs for controlling the CarMovement component.
    /// </summary>
    public double[] CurrentControlInputs
    {
        get { return Movement.CurrentInputs; }
    }

    /// <summary>
    /// The cached SpriteRenderer of this car.
    /// </summary>
    public SpriteRenderer SpriteRenderer
    {
        get;
        private set;
    }

    private Sensor[] sensors;
    private float timeSinceLastCheckpoint;
    #endregion

    #region Constructors
    void Awake()
    {
        //Cache components
        Movement = GetComponent<CarMovement>();
        SpriteRenderer = GetComponent<SpriteRenderer>();
        sensors = GetComponentsInChildren<Sensor>();
        pathFind = pathObject.GetComponent<PathFinding>();
    }
    void Start()
    {
        Movement.HitWall += Die;

        //Set name to be unique
        this.name = "Car (" + NextID + ")";
        this.InitializePath();
        sw = new Stopwatch();
        sw.Start();
    }

    //initialization of the A* path
    void InitializePath()
    {
        //Get the initial rotation of a car, later used for calculating an angle between car and goal node
        initialRotation = Movement.GetZAngle();

        //get a list of waypoint objects from PathFinding class
        waypoint = pathFind.StartFinding(Movement.GetPosition());

        //get an array of waypoint locations.
        waypoints = new Vector2[waypoint.Count];
        for (int i = 0; i < waypoint.Count; i++)
        {
            waypoints[i] = waypoint[i].position;
        }

        //create boundary lines, waypoints will be treated as lines to be passed rather than points to be reached
        linepath = new Path(waypoints, Movement.GetPosition());

        //initialize variables
        iterat = 0;
        goalPos = nextGoalPos;
        nextGoalPos = goalPos;
        angleToNMultipleWpts = 0f;

        //set goal position to the current waypoint
        goalPos = linepath.points[iterat];
        //set angle from the next waypoint to the one after
        angleToNextWpt = waypoint[iterat].angleToNextWaypt;
        //get an absolute value for an angle
        angleToNextWpt = Mathf.Abs(angleToNextWpt);
        //get next goal position to the next waypoint
        if (iterat < waypoints.Length - 1)
        {
            nextGoalPos = linepath.points[iterat + 1];
        }
        //get distance between current waypoint and next
        distBetweenWpt = Vector2.Distance(nextGoalPos, goalPos);

        updatedAngles = false;
    }

    #endregion

    #region Methods
    /// <summary>
    /// Restarts this car, making it movable again.
    /// </summary>
    public void Restart()
    {
        Movement.enabled = true;
        timeSinceLastCheckpoint = 0;

        foreach (Sensor s in sensors)
            s.Show();

        //Agent.Reset();
        this.enabled = true;
    }

    //method used to calculate vertical and horizontal inputs for the vehicle
    double[] CalculateInputs(double[] sensorOutput)
    {
        //initialize the inputs and find the velocity of the car
        double verticalInput = 0;
        double horizontalInput = 0;
        float velocity = Movement.GetVelocity();


        //find position and rotation of the car, calculate its direction
        Vector2 pos = Movement.GetPosition();
        Vector2 directions = new Vector2(0, 1);
        Quaternion rotations = Movement.GetRotation();
        directions = rotations * directions;
            

        //find distance and angle to the waypoint
        Vector2 targetDir = goalPos - pos;
        float waypointDist = Vector2.Distance(pos, goalPos);

        //find angle between car and target waypoint
        float angle = Vector2.Angle(directions, targetDir);
  
        //initialize normalized angle
        float normalizeAngle = 1f;

        //check if waypoint has been reached and update variables
        CheckWaypointReached(pos, waypointDist);

        //get distance from the front sensor
        double distanceCheck = sensorOutput[2];
        


        float normalizedVel = velocity / 20;


        UpdateAnglesAndStoppingDist(waypointDist, velocity);

        //if angle = 0 then normalized angle equals zero
        if (angle == 0)
                normalizeAngle = 0f;
        else   //otherwise angle gets normalized by dividing it by 90 and adding 0.1 tuning value to optimize it for calculating velocity
            {
                normalizeAngle = (Mathf.Abs(angle) + angleToNextWpt) / 90f + 0.1f;
                normalizeAngle = Mathf.Clamp(normalizeAngle, 0f, 0.9f); //clamp normalized angle between 0 and 0.9 as vertical input will be 1 - normalized ang, we do not want vertical input to be 0
            }

        //this sets the distance check to be waypoint distance instead of front sensor distance, this optimizes the speed for u turns or very sharp turns.
        if (distBetweenWpt < 5f && angleToNextWpt + angleToNMultipleWpts >= 40)
            distanceCheck = waypointDist + 2;

        
        //comparing distance check against velocity and angle of turns and setting the vertical input to optimal one
        if (distanceCheck > velocity * (normalizedVel + ((angleToNextWpt + angleToNMultipleWpts) / 90f)) && (waypointDist > stoppingDist))
        {
            verticalInput = 1f - normalizeAngle;
        }
        else if (velocity < 3)
        {
            verticalInput = 0.1f;
        }
        else
        {
            verticalInput = -1;
        }

        //if there is an obstacle very close to the front of the vehicle then try to stop and reverse
        if (sensorOutput[2] < 1f)
        {
            verticalInput = -1f;
        }

        //calculate the relative vector between vehicle and target waypoint
        Vector2 relativeVec = Movement.GetRelativeVec(goalPos);
            
        //set horizontal input
        horizontalInput = (relativeVec.x / relativeVec.magnitude);
            
        //if side sensors detect an obstacle very close to a car then try to make a sharp turn away from the obstacle
        if ((sensorOutput[4] < 0.7f && sensorOutput[4] < sensorOutput[0]) || sensorOutput[3] < 0.5f)
        {
            horizontalInput = 1;
        }
        if ((sensorOutput[0] < 0.7f && sensorOutput[0] < sensorOutput[4]) || sensorOutput[1] < 0.5f)
        {
            horizontalInput = -1;
        }



        //return horizontal (turn left/right intensity) and vertical  (acceleration/deceleration intensity) inputs
        double[] inputs = new double[2] { horizontalInput, verticalInput };
        return inputs;
    }

    void UpdateAnglesAndStoppingDist(float _waypointDist, float _velocity)
    {
        //if waypoint distance is lower than max stopping distance and angles haven't been updated yet
        if (_waypointDist < _velocity * 1.25 && updatedAngles == false)
        {
            int j = iterat + 1;
            float normalizedVel = _velocity / 20;
            int k = 0;
            //initialize variables
            stoppingDist = 0;
            angleToNMultipleWpts = 0;

            float angles = 0;
            float distBetweenAllCheckedPts = 0;
            updatedAngles = true;
            //Check next 3 waypoint angles and distances to measure stopping distance
            while (j <= iterat + 3 && j < linepath.points.Length)
            {
                angles += Mathf.Abs(waypoint[j].angleToNextWaypt);
                float dist = Vector2.Distance(linepath.points[iterat], linepath.points[j]);
                distBetweenAllCheckedPts += dist;
                //check whether stopping distance should be applied (when waypoints are close together and car is expected to have a high speed when entering a corner)
                if (_waypointDist * 2 - _velocity > (distBetweenAllCheckedPts * (1 - normalizedVel)) && angles > 80)
                {
                    stoppingDist = _waypointDist;
                    stoppingDist = (stoppingDist > 25) ? 25 : stoppingDist; //25 is max stopping distance
                }


                //check whether angles divided by distance are greater than 40 - k, if it is then divide sum of angles by two to optimize speed calculation
                dist = (dist < 1) ? 1 : dist;
                if (angles / dist > 40 - k)
                {
                    //if they are then increase the angle to next waypoints
                    angleToNMultipleWpts = angles / 2;
                    break;
                }
                j++;
                k += 10;
            }
        }
    }

    //method used to calculate next waypoints and their parameters if current checkpoint has been reached
    void CheckWaypointReached(Vector2 _pos, float _waypointDist)
    {
        //if passed the waypoiny boundary line or got close enought to waypoint them increment iteration and get next positions, angles, and distances
        if ((linepath.turnBounds[iterat].CrossedLine(_pos) || _waypointDist < 1) && iterat < linepath.points.Length - 1)
        {
            updatedAngles = false;
            iterat++;   //increment iteration
            if (iterat == linepath.points.Length - 1)
            {
                sw.Stop();
                print("Lap time: " + sw.Elapsed + " s");
            }
            angleToNextWpt = waypoint[iterat].angleToNextWaypt;     //get angle between current and next waypoint
            goalPos = linepath.points[iterat];  //set current (iter) waypoint target to new one based on iteration
            if(iterat < linepath.points.Length - 2)
            {
                nextGoalPos = linepath.points[iterat + 1];
            }

            distBetweenWpt = Vector2.Distance(nextGoalPos, goalPos);    //calculate distances between waypoints
            angleToNextWpt = Mathf.Abs(angleToNextWpt); //convert angle values to absolute
  


        }
    }

    // Unity method for normal update
    void Update()
    {
        timeSinceLastCheckpoint += Time.deltaTime;
    }

    // Unity method for physics update
    void FixedUpdate()
    {
        //Get control inputs from Agent
        if (!UseUserInput)
        {
            //Get readings from sensors
            double[] sensorOutput = new double[sensors.Length];
            for (int i = 0; i < sensors.Length; i++)
                sensorOutput[i] = sensors[i].Output;

            double[] controlInputs = this.CalculateInputs(sensorOutput);
            Movement.SetInputs(controlInputs);
        }

    }

    // Makes this car die (making it unmovable and stops the Agent from calculating the controls for the car).
    private void Die()
    {
        this.enabled = false;
        Movement.Stop();
        Movement.enabled = false;

        foreach (Sensor s in sensors)
            s.Hide();

        //Agent.Kill();
    }

    public void CheckpointCaptured()
    {
        timeSinceLastCheckpoint = 0;
    }
    #endregion


    private void OnDrawGizmos()
    {
        if (linepath != null)
        {
            linepath.DrawWithGizmos();
        }
        //Gizmos.DrawWireCube(transform.position, new Vector2(gridWorldSize.x, gridWorldSize.y));

        if (waypoints != null)
        {
            Gizmos.color = Color.black;
            //Gizmos.DrawLine(waypoints[0], transform.position);
            for (int i = 1; i < waypoints.Length; i++)
            {
                Gizmos.color = Color.black;
                Gizmos.DrawCube(waypoints[i], Vector3.one - new Vector3(0.9f, 0.9f, 0.9f));

                Gizmos.DrawLine(waypoints[i - 1], waypoints[i]);
            }

        }

    }
}
