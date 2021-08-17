using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class RobotUIController : MonoBehaviour
{
    public float current_angle0;
    public float current_angle2;
    public float desiredAngle;
    public float temp;

    private bool angleReached = true;

    public ServoMotor servo;

    [SerializeField]
    private Robot robot;
    [SerializeField]
    private Toggle toggleUpper;
    [SerializeField]
    private Toggle toggleLower;
    [SerializeField]
    private Text distanceText;
    [SerializeField]
    private Text headingText;
    [SerializeField]
    public ServoUIController upperLeg1Servo;
    [SerializeField]
    public ServoUIController lowerLeg1Servo;
    [SerializeField]
    public ServoUIController upperLeg2Servo;
    [SerializeField]
    public ServoUIController lowerLeg2Servo;
    [SerializeField]
    public ServoUIController upperLeg3Servo;
    [SerializeField]
    public ServoUIController lowerLeg3Servo;
    [SerializeField]
    public ServoUIController upperLeg4Servo;
    [SerializeField]
    public ServoUIController lowerLeg4Servo;
    [SerializeField]
    private ServoProfile profile;

    private void Awake()
    {
    