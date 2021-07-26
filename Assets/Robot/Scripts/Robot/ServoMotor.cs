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

        IsMotorE