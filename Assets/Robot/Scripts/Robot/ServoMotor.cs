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
     