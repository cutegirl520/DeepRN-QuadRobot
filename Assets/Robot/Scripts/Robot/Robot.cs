using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Robot : MonoBehaviour
{
    private Robot robot;

    public float forwardDirectionOffset = 0;

    public Leg[] legs = new Leg[4];

    private Vector3 initialDirection;
    private Vector3 initialPosition;

    /// <summary>
    /// Store initial position and rotation.
    /// </summary>
    private void Awake()
    {
        initialDirection = GetForwardDirection();
        initialPosition = transform.position;
    }

    public void Update()
    {

       // robot.legs[1].lowerLeg.SetAngle(45);
    }

    /// <summary>
    /// Get look direction of the robot.
    /// </summary>
    private Vector3 GetForwardDirection()
    {
        return Vector3.ProjectOnPlane(Quaternion.AngleAxis(forwardDirectionOffset, transform.up) * transform.forward, Vector3.up);
    }

    /// <summary>
    /// Get current robot heading in degrees.
    /// </summary>
    public float GetHeading()
    {
        return Vector3.SignedAngle(initialDirection, GetForwardDirection(), Vector3.up);
    }

    /// <summary>
    /// Get distance walked forward.
    /// </summary>
    public float GetDistance()
    {
        Vector3 movementVector = Vector3.Project((tr