using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[ExecuteInEditMode]
[RequireComponent(typeof(Rigidbody))]
public class ServoMotor : MonoBehaviour
{
    public bool isClockwise = true;

    private Robot robot; //added

    [Space]
    [SerializeField]
    private float targetAngle = 0;

    [Header("Geometry")]    //Used to add a header above fields in inspector
    public Rigidbody servoBase; //this is robot body
    public Vector3 anchor = Vector3.zero;
    public Vector3 axis = Vector3.right;
    public float minAngle = -180;
    public float maxAngle = 180;
    public float angleGizmoOffset = 0;

    // Joint limits should be set a little wider than servo limits
    // Otherwise unexpected behaviour on the boundaries occures
    private const float jointLimitsOffset = 1f;

    [Header("Physics")]
    [SerializeField]
    private ServoProfile profile;

    [Header("Feedback")]
    [SerializeField]
    private PIDProfile positionRegulatorProfile;
    [SerializeField]
    private PIDProfile velocityRegulatorProfile;

    private PIDRegulator positionRegulator;
    private PIDRegulator velocityRegulator;

    private HingeJoint joint;
    private new Rigidbody rigidbody;
    private Vector3 correctedAxis;
    private Vector3 axisRelativeToBase;
    private Vector3 zeroDirection;
    
    public float Range
    {
        get
        {
            return maxAngle - minAngle;
        }
    }

    /// <summary>
    /// Fixing servo is similar to replacing HingeJoint with a FixedJoint.
    /// </summary>
    private bool _isFixed;
    public bool IsFixed
    {
        get
        {
            return _isFixed;
        }
        set
        {
            if (_isFixed == value || joint == null) return;
            _isFixed = value;

            // Fix servo position
            if (_isFixed)
            {
                targetAngle = GetServoAngle();
                var jointAngle = GetJointAngle();
                joint.limits = new JointLimits() { bounceMinVelocity = float.MaxValue, min = jointAngle - 0.001f, max = jointAngle };
            }
            // Unfix servo position
            else
            {
                joint.limits = new JointLimits() { bounceMinVelocity = float.MaxValue, min = minAngle - jointLimitsOffset, max = maxAngle + jointLimitsOffset };
            }
        }
    }

    /// <summary>
    /// An interface to access joint.useMotor. The value is cached to increase performance,
    /// because accessing joint.useMotor normally takes some processing time.
    /// </summary>
    private bool _isMotorEnabled;
    public bool IsMotorEnabled
    {
        get
        {
            return _isMotorEnabled;
        }
        set
        {
            if (_isMotorEnabled == value || joint == null) return;
            joint.useMotor = _isMotorEnabled = value;
        }
    }

    /// <summary>
    /// Initialize servo.
    /// </summary>
    private void Awake()
    {
        if (Application.isPlaying)
        {
            correctedAxis = GetCorrectedAxis();
            rigidbody = GetComponent<Rigidbody>();
            positionRegulator = new PIDRegulator(positionRegulatorProfile);
            velocityRegulator = new PIDRegulator(velocityRegulatorProfile);

            CreateJoint();

            if (!profile || !positionRegulatorProfile)
            {
                Debug.Log("Servo " + name + " will not work properly, because it is not fully configured!");
            }
        }
    }

    /// <summary>
    /// Creates HingeJoint.
    /// </summary>
    private void CreateJoint()
    {
        //gameObject is a Base class for all entities in Unity scenes, Adds a component class named className to the game object.
        joint = gameObject.AddComponent<HingeJoint>();
        joint.connectedBody = servoBase;
        joint.axis = correctedAxis;
        joint.anchor = anchor;
        joint.useLimits = true;
        joint.limits = new JointLimits() { bounceMinVelocity = float.PositiveInfinity, min = minAngle - jointLimitsOffset, max = maxAngle + jointLimitsOffset };

        IsMotorEnabled = true;
        IsFixed = false;
        axisRelativeToBase = GetAxisRelativeToBase();
        zeroDirection = GetJointDirection();
        targetAngle = JointAngleToServoSpace(GetJointAngle());
    }

    /// <summary>
    /// If axis vector equals to 0, 0, 0, by default Unity uses X axis.
    /// </summary>
    private Vector3 GetCorrectedAxis()
    {
        //right means (1,0,0)
        return axis.magnitude == 0 ? Vector3.right : axis.normalized;
    }

    /// <summary>
    /// Calculates axis direction relative to the base object.
    /// </summary>
    /// <returns></returns>
    private Vector3 GetAxisRelativeToBase()
    {
        //Transform represents Position, rotation and scale of an object. The rotation of the transform in world space stored as a Quaternion.
        Vector3 res = transform.rotation * correctedAxis;
        if (servoBase)
        {
            //Quaternions are used to represent rotations.
            res = Quaternion.Inverse(servoBase.transform.rotation) * res;
        }
        return res;
    }

    /// <summary>
    /// Execute every frame.
    /// </summary>
    private void Update()
    {
        // In editor mode, joint look direction is always zero direction
        // Returns true in the Unity editor when in play mode.
        if (!Application.isPlaying)
        {
            correctedAxis = GetCorrectedAxis();
            zeroDirection = GetJointDirection();
            axisRelativeToBase = GetAxisRelativeToBase();
        }
    }

    /// <summary>
    /// Servo motor physics.
    /// </summary>
    private void FixedUpdate()
    {
        if (!IsMotorEnabled || IsFixed || !positionRegulatorProfile || !profile) return;

        // Run servo motor
        if (IsMotorEnabled)
        {
            // Angle regulator controls servo velocity based on the position error
            var targetVelocity = positionRegulator.Run(ServoAngleToJointSpace(targetAngle), GetJointAngle(), Time.fixedDeltaTime);

            //Clamps target velocity according to servo profile
            targetVelocity = Mathf.Clamp(targetVelocity, -profile.maxVelocity, profile.maxVelocity);

            // Unity is quite bad at keeping target velocity, so we might use an extra regulator, which 
            // takes velocity error as an input. Even though we write its output value to joint.motor.targetVelocity,
            // it works more like a torque control.

            //Basically this line calculates the velocity error. Will add later on.
            var velocityCorrection = velocityRegulator.Run(targetVelocity, GetServoVelocity(), Time.fixedDeltaTime);
            velocityCorrection = Mathf.Clamp(velocityCorrection, -500, 500);
            //joint motor applies maxForce Available and sets target vel
            joint.motor = new JointMotor() { force = profile.maxForce, freeSpin = false, targetVelocity = targetVelocity + velocityCorrection };
        }
    }

    /// <summary>
    /// Get angular velocity in degrees per second.
    /// </summary>
    public float GetServoVelocity()
    {
        // We have to calculate servo velocity manually, because HingeJoint.velocity shows target velocity 
        // and not the actual velocity of the rigidbody
        Vector3 velocity = rigidbody.angularVelocity * Mathf.Rad2Deg;
        
        //we want to get servo velocity thats why we subtract from robot body.
        if (servoBase) velocity = velocity - servoBase.angularVelocity;
        //transforms vector from world to local space
        velocity = transform.InverseTransformVector(velocity);

        //returns dot product of 2 vectors. Angular velocity is angular displacement relative to origin
        return Vector3.Dot(velocity, correctedAxis);
    }

    /// <summary>
    /// Get angle.
    /// </summary>
    public float GetServoAngle()
    {
        //The output of this function is determined by clockwise or anticlockwise servo rotation
        return JointAngleToServoSpace(GetJointAngle());
    }

    /// <summary>
    /// Get joint angle in degrees.
    /// </summary>
    // Changed to public
    private float GetJointAngle()
    {
        // We have to calculate joint angle manually, because HingeJoint.angle is broken since Unity 5.2 
        //Returns the signed angle in degrees between from and to, about axis.
        //The smaller of the two possible angles between the two vectors is returned, therefore the result will never be greater than 180 degrees or smaller than -180 degrees.
        return Vector3.SignedAngle(zeroDirection, GetJointDirection(), axisRelativeToBase);
    }

    /// <summary>
    /// Look direction of the joint. Used in the angle calculations by Unity.
    /// </summary>
    private Vector3 GetJointDirection()
    {
        // Direction from the unscaled joint center to the transform position
        Vector3 dir = transform.position - GetUnscaledJointPosition();

        // Or just use one