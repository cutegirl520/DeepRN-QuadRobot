using UnityEngine;
using UnityEngine.UI;

using System;
using System.IO;
using System.Net.Sockets;
using System.Collections;
using System.Text.RegularExpressions;

public class TCPUnity : MonoBehaviour
{
	TcpClient tcp_socket;
	NetworkStream net_stream;
	StreamWriter socket_writer;
	StreamReader socket_reader;

	public String host = "localhost";
	public Int32 port = 50000;

	internal Boolean connected = false;
	internal Boolean canSend = true;

	internal String sendData = "";
	internal String receivedData = "";

	public InputField xInput;
	public InputField yInput;
	public InputField zInput;

	public float floatInput;
	public int intInput;

	public Text receivedTxt;
	public Text statusTxt;

	public void Connect()
	{
		if (!connected)
		{
			try
			{
				tcp_socket = new TcpClient(host, port);

				net_stream = tcp_socket.GetStream();
				socket_writer = new StreamWriter(net_stream);
				socket_reader = new StreamReader(net_stream);

				connected = true;
