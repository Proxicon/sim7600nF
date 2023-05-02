using System;
using System.IO.Ports;
using System.Text;
using System.Diagnostics;
using System.Device.Gpio;
using System.Threading;
using nanoFramework.Runtime.Events;
using TinyGPSPlusNF;
using System.Text.RegularExpressions;
using System.Collections;
using nanoFramework.Json;

namespace sim7600x
{
    public class sim7600
    {
        private readonly SerialPort _serial;
        private readonly StringBuilder _resultBuffer = new StringBuilder();
        private readonly AutoResetEvent _serialDataFinished = new AutoResetEvent(false);
        private string _lastResult = "";
        private string _lastResultPostReturn = "";
        private readonly string _apn;
        private int _failures = 0;
        private int _returnBufferCounter = 0;
        private bool _isConnected = false;

        private static TinyGPSPlus s_gps;

        // GPIO init
        //private readonly GpioPin _batteryStatus; // Pin 35 for battery status, only works when not on USB aka, running on battery
        private readonly GpioPin _ledPower;      // Pin 12 for power led 

        // GPIO Modem
        private readonly GpioPin _MODEM_PWRKEY;  // Pin 4
        private readonly GpioPin _MODEM_DTR;     // Pin 32
        private readonly GpioPin _MODEM_RI;      // Pin 33
        private readonly GpioPin _MODEM_FLIGHT;  // Pin 25
        private readonly GpioPin _MODEM_STATUS;  // Pin 34

        // Serial interface - SPI
        private readonly GpioPin _SD_MISO;       //Pin 2
        private readonly GpioPin _SD_MOSI;       //Pin 15
        private readonly GpioPin _SD_SCLK;       //Pin 14
        private readonly GpioPin _SD_CS;         //Pin 13

        // API Auth
        private string _authToken;

        // public int pwkkey;
        // public int rstkey;
        // public int poweronkey;
        public sim7600(string apn, string gprsuser, string gprspass, string portName, int pwkkey, int flight, int ledpin)
        {
            _apn = apn;

            // init led & turn it off
            _ledPower = new GpioController().OpenPin(ledpin, PinMode.Output);
            _ledPower.Write(PinValue.Low);

            // Initialize the modem power and check network connection
            _MODEM_PWRKEY = new GpioController().OpenPin(pwkkey);
            _MODEM_PWRKEY.SetPinMode(PinMode.Output);

            /* Enable GPS for recieving AT commands
               For SIM7600E-H-M2/SIM7600SA-H-M2/SIM7600A-H-M2 module, GPS started should be decided
               by the physical switch of GPS flight mode in the module firstly. Open the switch, GPS will be
               started automatically, then you can open or close gps by AT command, otherwize, GPS could not
               be started in any way.it will report +CME ERROR:GPS flight mode enable
            */

            if (flight > 0)
            {
                _MODEM_FLIGHT = new GpioController().OpenPin(flight);
                _MODEM_FLIGHT.SetPinMode(PinMode.Output);
                _MODEM_FLIGHT.Write(PinValue.High);
            }


            // Create TinyGPS
            //TinyGPSPlus gps = new();

            // Create serial hardware interface & attach event handler
            _serial = new SerialPort(portName, 115200, Parity.None, 8, StopBits.One);
            _serial.Handshake = Handshake.RequestToSend;

            _serial.DataReceived += SerialOnDataReceived;

            _serial.Open();


            // check automatice time & timezome update setting
            AutomaticTimeandTimezoneUpdate();

            /*
                Turn on the sim chip, sim chip stays online after flashing new firmware delaying the startup sequence
                we add a check first to see if the network is connected after module reboot or new firmware flashed
                before turning it off and back on.
            */


            // check connectivity state
            bool isConnected = NetworkisConnected();

            if (isConnected)
            {
                Debug.WriteLine("Modem is connected to network, no need to cycle power...\n");

                /* indicate modem can be controlled*/
                _ledPower.Toggle();

                // GprsConnect(apn, gprsuser, gprspass);
                // ModemHTTPTesting();
            }
            else
            {
                Debug.WriteLine("Modem network connection not detected, cycling chip power..\n");

                if (pwkkey > 0)
                {
                    _MODEM_PWRKEY.Write(PinValue.High);
                    Thread.Sleep(300);
                    _MODEM_PWRKEY.Write(PinValue.Low);

                    /* allow modem to wake well and to be ready for use */
                    Thread.Sleep(10000);

                    /* indicate modem can be controlled*/
                    _ledPower.Toggle();

                    //ModemStartupTesting(apn, gprsuser, gprspass);
                    //ModemHTTPTesting();
                    //ModemInit();
                    //GprsConnect(apn, gprsuser, gprspass);
                }
            }
        }

        /// <summary>
        /// Initializes the GSM modem.
        /// </summary>
        /// <param name="serialPort">The SerialPort instance used to communicate with the modem.</param>
        public void ModemInit()
        {
            // Reset the modem
            SendCommand("ATZ\r", true);
            SendEndOfDataCommand();

            // Check the modem status
            SendCommand("AT\r", true);
            SendEndOfDataCommand();

            // Configure the modem settings
            SendCommand("ATE0\r", true); // Disable echo mode
            SendEndOfDataCommand();
            SendCommand("AT+CMEE=2\r", true); // Enable verbose error messages
            SendEndOfDataCommand();
            SendCommand("AT+CMGF=1\r", true); // Set SMS message format to text mode
            SendEndOfDataCommand();

            // Initialize the SIM card
            SendCommand("AT+CPIN?\r", true); // Check if SIM card is present
            SendEndOfDataCommand();
            SendCommand("AT+CPIN=\"00000\"\r", true); // Enter SIM card PIN (if required)
            SendEndOfDataCommand();

            // Wait for network registration
            SendCommand("AT+CREG?\r", true); // Check network registration status
            while (_lastResult.IndexOf("+CREG: 0,1") == -1 && _lastResult.IndexOf("+CREG: 0,5") == -1)
            {
                Thread.Sleep(1000);
                SendCommand("AT+CREG?\r", true);
                SendEndOfDataCommand();
            }
        }

        /// <summary>
        /// This method powers off the modem, waits for a short period, and then powers it back on again. It also waits for 
        /// the modem to wake up and be ready for use before reinitializing it using the ModemInit method
        /// </summary>
        public void PowerCycleModem()
        {
            // Power off the modem
            _MODEM_PWRKEY.SetPinMode(PinMode.Output);
            _MODEM_PWRKEY.Write(PinValue.High);
            Thread.Sleep(300);
            _MODEM_PWRKEY.Write(PinValue.Low);

            // Allow modem to wake and be ready for use
            Thread.Sleep(10000);

            // Reinitialize the modem
            ModemInit();
        }

        /// <summary>
        /// Establishes a GPRS connection using the specified APN and login credentials.
        /// </summary>
        /// <param name="apn">The Access Point Name (APN) of the cellular network.</param>
        /// <param name="gprsUser">The user name (if required) for GPRS authentication.</param>
        /// <param name="gprsPass">The password (if required) for GPRS authentication.</param>
        public void GprsConnect(string apn, string gprsUser = null, string gprsPass = null)
        {
            Debug.WriteLine("----------------(GprsConnect)----------------");

            // Enable GPRS
            SendCommand("AT+CGATT=1\r", true);
            SendEndOfDataCommand();

            // Set APN
            SendCommand($"AT+CGDCONT=1,\"IP\",\"{apn}\"\r", true);
            SendEndOfDataCommand();

            // Set login credentials (if provided)
            if (gprsUser != null && gprsPass != null)
            {
                SendCommand($"AT+CGAUTH=1,0,\"{gprsUser}\",\"{gprsPass}\"\r", true);
                SendEndOfDataCommand();
            }

            // Activate PDP context
            SendCommand("AT+CGACT=1,1\r", true);
            SendEndOfDataCommand();

            // Check PDP context status
            SendCommand("AT+CGACT?\r", true);
            while (_lastResult.IndexOf("+CGACT: 1,0") != -1)
            {
                Thread.Sleep(1000);
                SendCommand("AT+CGACT?\r", true);
            }
        }

        public void SimCheck()
        {
            int restartCounter = 5;
            int atAttempts = 0;

            SendCommand("AT\r", true); //Echo OFF

            while (_lastResult.IndexOf("OK") < 0)
            {
                if (atAttempts > restartCounter)
                {
                    Debug.WriteLine($"Restarting Modem after {restartCounter} SimCheck Attempts...");

                    TogglePower();
                    atAttempts = 0;
                }

                Thread.Sleep(1000);

                SendCommand("AT\r", true); //Echo OFF

                atAttempts++;
            }
        }

        public void TogglePower()
        {
            _MODEM_PWRKEY.Write(PinValue.High);
            Thread.Sleep(2500);
            _MODEM_PWRKEY.Write(PinValue.Low);

            // Startup grace
            Thread.Sleep(10000);
        }

        public string GetInternationMobileEquipmentIdentifier()
        {
            SendCommand("AT+GSN", true);
            return _lastResult;
        }

        private void SerialOnDataReceived(object sender, SerialDataReceivedEventArgs serialDataReceivedEventArgs)
        {
            // Check if Chars are received
            if (serialDataReceivedEventArgs.EventType == SerialData.Chars)
            {
                // Create new buffer
                var readBuffer = new byte[_serial.BytesToRead];

                // Read bytes from buffer
                _serial.Read(readBuffer, 0, readBuffer.Length);

                // Encode to string
                _resultBuffer.Append(new String(Encoding.UTF8.GetChars(readBuffer)));

                if (_resultBuffer.Length > 0 && _resultBuffer[_resultBuffer.Length - 1] == 10)
                {
                    _lastResult = _resultBuffer.ToString();

                    Debug.WriteLine("SerialOnDataReceived._lastResult" + _lastResult);

                    _resultBuffer.Clear();

                    _serialDataFinished.Set();
                }
            }
        }

        public void InitializeModem()
        {
            SendCommand("ATE0\r", true); //Echo ON

            // First check if we are already connected to gprs EPS in order to bypass connection sequence
            Debug.WriteLine("Checking if previously connected to network...");
            bool connectionDetected = NetworkGetRegistrationState();
            Debug.WriteLine($"Checking if previously connected to network returned: {connectionDetected}");

            // Enable
            ReportMobileEquipmentError();

            if (connectionDetected == true)
            {
                Debug.WriteLine("Detected network connection, bypassing connection sequence...");

            }
            else
            {
                Debug.WriteLine("Starting network connection sequence...");

                PreferredModeSelectionAuto();
                // NetworkStopTCPIPService();

                // Set APN details 
                // NetworkDefinePDPConext("myMTN");
                // NetworkSetAuthType("","");9

                // Initial settings
                // Configure TCP parameters
                NetworkSetupTCPUDPClientSocketConnection();
                NetworkSetupSendingMode();
                NetworkSetupSocketParamaters();
                NetworkSetTCPIP_Timeout();

                // Start the socket service
                // You must execute AT+NETOPEN before any other TCP/UDP related operation

                // This activates and attaches to the external PDP context that is tied
                // to the embedded context for TCP/IP (ie AT+CGACT=1,1 and AT+CGATT=1)
                // Response may be an immediate "OK" followed later by "+NETOPEN: 0".
                // We to ignore any immediate response and wait for the
                // URC to show it's really connected.
                NetworkStartSocketService();

                RequestInternationalMobileSubscriberIdentity();
            }
        }

        /// <summary>
        /// Sends an AT command to the device and optionally waits for a response.
        /// </summary>
        /// <param name="command">The AT command to send.</param>
        /// <param name="waitForResponse">A value indicating whether to wait for a response.</param>
        /// <param name="timeout">The maximum amount of time to wait for a response (in milliseconds).</param>
        public void SendCommand(string command, bool waitForResponse = false, int timeout = 100)
        {
            // Log the command being sent to the device
            Debug.WriteLine($"AT+Comm = [{command}]");

            // Reset the ManualResetEvent used to signal the completion of the data transfer
            _serialDataFinished.Reset();

            // Clear the _lastResult variable, indicating that no response has been received yet
            _lastResult = "";

            // Convert the command string to a byte array
            var writeBuffer = Encoding.UTF8.GetBytes(command);

            // Send the command to the device
            _serial.Write(writeBuffer, 0, writeBuffer.Length);

            // If waiting for a response, wait for the response signal from the device
            if (waitForResponse)
            {
                _serialDataFinished.WaitOne(timeout, false);
            }
        }

        /// <summary>
        /// Sends the end-of-data command to the module.
        /// </summary>
        public void SendEndOfDataCommand()
        {
            _serialDataFinished.Reset();

            var endChar = new byte[1];
            endChar[0] = 26;

            _serial.Write(endChar, 0, 1);

            _serialDataFinished.WaitOne(1000, false);
        }

        public void Test()
        {
            SendCommand("AT\r", true);
            SendEndOfDataCommand();
        }

        /// <summary>
        /// Resets the sim module.
        /// </summary>
        public void ResetModule()
        {
            Debug.WriteLine("----------------(ResetModule)----------------");

            // Test Command
            SendCommand("AT+CRESET=?\r", true);
            SendEndOfDataCommand();
            // Exec Command
            SendCommand("AT+CRESET\r", true);
        }

        public void DisplayCurrentConfiguration()
        {
            Debug.WriteLine("----------------(DisplayCurrentConfiguration)----------------");

            SendCommand("AT&V\r", true);
            SendEndOfDataCommand();
        }

        public void RequestModelIdentification()
        {
            Debug.WriteLine("----------------(RequestModelIdentification)----------------");

            SendCommand("AT+CGMM\r", true);
            SendEndOfDataCommand();
        }

        public void ReportMobileEquipmentError()
        {
            SendCommand("AT+CMEE=2\r", true);
            SendEndOfDataCommand();
        }

        public void AutomaticTimeandTimezoneUpdate()
        {
            Debug.WriteLine("----------------(AutomaticTimeandTimezoneUpdate)----------------");

            // Test command
            SendCommand("AT+CTZU=?\r", true);
            SendEndOfDataCommand();

            // Read Command
            SendCommand("AT+CTZU?\r", true);
            if (_lastResult.IndexOf("+CTZU: 1") >= 0)
            {
                Debug.WriteLine("NO Need to update CTZU, already set to 1");
                SendEndOfDataCommand();
            }
            else
            {
                // Write Command
                SendCommand("AT+CTZU=1\r", true);
                SendEndOfDataCommand();
            }
        }

        public void ReadtheVoltageValueOfThePowerSupply()
        {
            Debug.WriteLine("----------------(ReadtheVoltageValueOfThePowerSupply)----------------");

            SendCommand("AT+CBC\r", true);
            SendEndOfDataCommand();
        }

        /*This command is used to configure GPS, GLONASS, BEIDOU and QZSS support mode. And DPO(Dynamic power optimization) status
        Module should reboot to take effec*/
        public void ConfigureGNSSSupportMode()
        {
            Debug.WriteLine("----------------(ConfigureGNSSSupportMode)----------------");

            // Read Command
            SendCommand("AT+CGNSSMODE?\r", true);

            // Only trigger write action if the setting is not 1,1
            if (_lastResult.IndexOf("+CGNSSMODE: 1,1") > 0)
            {
                Debug.WriteLine("+CGNSSMODE: 1,1 detected, no need to trigger write");
                SendEndOfDataCommand();

            }
            else
            {
                // Write Command
                SendCommand("AT+CGNSSMODE=1,1\r", true);
                SendEndOfDataCommand();
            }
        }

        public void StartStopGpsSession(int State = 1)
        {
            Debug.WriteLine("----------------(StartStopGpsSession)----------------");

            // Test Command
            SendCommand("AT+CGPS=?\r", true);
            SendEndOfDataCommand();

            // Read Command
            SendCommand("AT+CGPS?\r", true);

            // Only trigger write action if the setting is not 1,1
            if (_lastResult.IndexOf("+CGPS: 1,1") > 0)
            {
                Debug.WriteLine($"No need to set detected +CGPS: 1,1 ");
                SendEndOfDataCommand();
            }
            else
            {
                // Write Command
                SendCommand("AT+CGPS=1,1\r", true);
                SendEndOfDataCommand();

                Thread.Sleep(5000);
            }
        }

        public string GetGPSFixedPositionInformation()
        {
            Debug.WriteLine("----------------(GetGPSFixedPositionInformation)----------------");

            SendCommand("AT+CGPSINFO\r", true);
            SendEndOfDataCommand();

            string response = null;

            var match = Regex.Match(_lastResult, @"\+CGPSINFO: (.*)");
            if (match.Success)
            {
                response = match.Groups[1].Value;
                Debug.WriteLine("The _lastResult response was: " + response);
            }
            else
            {
                Debug.WriteLine("Failed to retrieve GPS information.");
            }

            return response;
        }


        public void QuerySignalQuality()
        {
            Debug.WriteLine("----------------(QuerySignalQuality)----------------");

            // Test Command
            SendCommand("AT+CSQ=?\r", true);
            SendEndOfDataCommand();

            // Execution Command
            SendCommand("AT+CSQ\r", true);
            SendEndOfDataCommand();
        }

        public void OperatorSelection()
        {
            Debug.WriteLine("----------------(OperatorSelection)----------------");

            // Test Command
            SendCommand("AT+COPS=?\r", true);
            SendEndOfDataCommand();

            // Read Command
            SendCommand("AT+COPS?\r", true);
            SendEndOfDataCommand();

            // Write Command
            SendCommand("AT+COPS=0\r", true);
            SendEndOfDataCommand();
        }

        public void RequestInternationalMobileSubscriberIdentity()
        {
            Debug.WriteLine("----------------(RequestInternationalMobileSubscriberIdentity)----------------");

            // Test Command
            SendCommand("AT+CIMI=?\r", true);
            SendEndOfDataCommand();

            // Write Command
            SendCommand("AT+CIMI\r", true);
            SendEndOfDataCommand();
        }

        public void PreferredModeSelectionAuto()
        {
            Debug.WriteLine("----------------(PreferredModeSelectionAuto)----------------");

            // Test Command
            SendCommand("AT+CNMP=?\r", true);
            SendEndOfDataCommand();

            // Read Command
            SendCommand("AT+CNMP?\r", true);

            // Validate setting Command
            if (_lastResult.IndexOf("+CNMP: 2") > 0)
            {
                Debug.WriteLine("+CNMP == 2, no need to set it again");
                SendEndOfDataCommand();
            }
            else
            {
                // Write Command
                // 2 auto, 13 GSM only, 38 LTE only
                SendCommand("AT+CNMP=0\r", true);
                SendEndOfDataCommand();
            }
        }

        public void NetworkStopTCPIPService()
        {
            Debug.WriteLine("----------------(NetworkStopTCPIPService)----------------");

            /*SendCommand("AT+CIPCLOSE\r", true);
            SendEndOfDataCommand();*/

            SendCommand("AT+NETCLOSE\r", true);
            SendEndOfDataCommand();
        }

        public void NetworkSetAuthType(string User = "", string Password = "0")
        {
            Debug.WriteLine("----------------(NetworkSetAuthType)----------------");

            // Test Command
            SendCommand("AT+CGAUTH=?\r", true);
            SendEndOfDataCommand();

            // Read Command
            SendCommand("AT+CGAUTH?\r", true);
            SendEndOfDataCommand();

            // Write Command
            SendCommand($"AT+CGAUTH=1,0,\"{User}\",\"{Password}\"\r", true);
            SendEndOfDataCommand();

            // Execute Command
            SendCommand("AT+CGAUTH\r", true);
            SendEndOfDataCommand();

            // Read Command
            SendCommand("AT+CGAUTH?\r", true);
            SendEndOfDataCommand();

        }

        public void NetworkDefinePDPConext(string APN = "internet")
        {
            Debug.WriteLine("----------------(NetworkDefinePDPConext)----------------");

            // Test command
            SendCommand("AT+CGDCONT=?\r", true);
            SendEndOfDataCommand();

            // Read command
            SendCommand("AT+CGDCONT?\r", true);
            SendEndOfDataCommand();

            // Write command
            SendCommand($"AT+CGDCONT=1,\"IP\",\"{_apn}\",\"0.0.0.0\",0,0\r", true);
            SendEndOfDataCommand();

            // Execute command
            SendCommand("AT+CGDCONT\r", true);
            SendEndOfDataCommand();
        }

        public void NetworkActivatePDPContext()
        {
            Debug.WriteLine("----------------(NetworkActivatePDPContext)----------------");

            // Test command
            SendCommand("AT+CGACT=?\r", true);
            SendEndOfDataCommand();

            // Read command
            SendCommand("AT+CGACT?\r", true);
            SendEndOfDataCommand();

            // Write command
            SendCommand("AT+CGACT=1,1\r", true);
            SendEndOfDataCommand();

        }

        public void ModemStartupTesting(string apn, string username = null, string password = null, string protocol = "TCP", string server = null, int port = 0)
        {
            Debug.WriteLine("----------------(ModemStartupTesting)----------------");

            // Query modem's manufacturer identification
            SendCommand("AT+CGMI\r", true);
            SendEndOfDataCommand();

            // Query modem's model identification
            SendCommand("AT+CGMM\r", true);
            SendEndOfDataCommand();

            // Check if modem is registered on the network
            SendCommand("AT+CREG?\r", true);
            SendEndOfDataCommand();

            // Set the APN
            SendCommand($"AT+CGDCONT=1,\"IP\",\"{apn}\",\"0.0.0.0\",0,0\r", true);
            SendEndOfDataCommand();

            // Set the username and password (if required)
            if (!string.IsNullOrEmpty(username) && !string.IsNullOrEmpty(password))
            {
                SendCommand($"AT+CGAUTH=1,1,\"{username}\",\"{password}\"\r", true);
                SendEndOfDataCommand();
            }

            // Enable network registration
            SendCommand("AT+CREG=1\r", true);
            SendEndOfDataCommand();

            // Check network registration status
            SendCommand("AT+CREG?\r", true);
            SendEndOfDataCommand();

            // Open the GPRS context
            SendCommand("AT+CGACT=1,1\r", true);
            SendEndOfDataCommand();

            // Check GPRS context activation status
            SendCommand("AT+CGACT?\r", true);
            SendEndOfDataCommand();

            // Check the IP address of the PDP context
            SendCommand("AT+CGPADDR\r", true);
            SendEndOfDataCommand();

            // Connect to the TCP or SSL client
            if (!string.IsNullOrEmpty(server) && port > 0)
            {
                SendCommand($"AT+CIPOPEN=\"{protocol}\",\"{server}\",{port}\r", true);
                SendEndOfDataCommand();

                // Send data to the server (if required)
                // AT+CIPSEND
                // ...

                // Close the connection
                SendCommand("AT+CIPCLOSE\r", true);
                SendEndOfDataCommand();
            }
        }

        public void ModemHTTPTesting()
        {
            Debug.WriteLine("----------------(ModemHTTPTesting)----------------");

            // Initialize HTTP service
            SendCommand("AT+HTTPINIT\r", true);
            SendEndOfDataCommand();

            // Set URL
            SendCommand("AT+HTTPPARA=\"URL\",\"http://sim.proxicon.co.za/\"\r", true);
            SendEndOfDataCommand();

            // Set HTTP method to GET
            SendCommand("AT+HTTPPARA=\"CID\",1\r", true);
            SendEndOfDataCommand();

            // Start GET request
            SendCommand("AT+HTTPACTION=0\r", true);
            SendEndOfDataCommand();

            // Read response
            SendCommand("AT+HTTPREAD=0,1000\r", true);
            SendEndOfDataCommand();

            // Close HTTP service
            SendCommand("AT+HTTPTERM\r", true);
            SendEndOfDataCommand();
        }

        public void TestPDPContext()
        {
            Debug.WriteLine("----------------(TestPDPContext)----------------");

            // Define the PDP context
            NetworkDefinePDPConext();

            // Activate the PDP context
            NetworkActivatePDPContext();

            // Test the connection by sending an HTTP GET request
            SendCommand("AT+HTTPPARA=\"URL\",\"http://www.example.com\"\r", true);
            SendEndOfDataCommand();

            SendCommand("AT+HTTPACTION=0\r", true);
            SendEndOfDataCommand();

            // Read HTTP response
            SendCommand("AT+HTTPREAD\r", true);
            SendEndOfDataCommand();
        }

        public void Ping(string hostname, int count, int interval, int packetsize, int timeout, int ttl, int vrf)
        {
            Debug.WriteLine("----------------(Ping)----------------");

            string command = $"AT+CPING=\"{hostname}\",{count},{interval},{packetsize},{timeout},{ttl},{vrf}\r";
            SendCommand(command, true);
            SendEndOfDataCommand();
        }

        // Select TCP/IP application mode (command mode)
        public void NetworkSetupTCPUDPClientSocketConnection()
        {
            Debug.WriteLine("----------------(NetworkSetupTCPUDPClientSocketConnection)----------------");

            // Test Command
            SendCommand("AT+CIPMODE=?\r", true);
            SendEndOfDataCommand();

            // Read Command
            SendCommand("AT+CIPMODE?\r", true);
            if (_lastResult.IndexOf("+CIPMODE: 0") > 0)
            {
                Debug.WriteLine("+CIPMODE == 0, no need to set it again");
                SendEndOfDataCommand();
            }
            else
            {
                // Write Command
                SendCommand("AT+CIPMODE=0\r", true);
                SendEndOfDataCommand();
            }
        }

        // Set Sending Mode - send without waiting for peer TCP ACK
        public void NetworkSetupSendingMode()
        {
            Debug.WriteLine("----------------(NetworkSetupSendingMode)----------------");

            // Test Command
            SendCommand("AT+CIPSENDMODE=?\r", true);
            SendEndOfDataCommand();

            // Read Command
            SendCommand("AT+CIPSENDMODE?\r", true);
            if (_lastResult.IndexOf("+CIPSENDMODE: 0") > 0)
            {
                Debug.WriteLine("+CIPSENDMODE = 0, no need to send it again");
                SendEndOfDataCommand();
            }
            else
            {
                // Write Command
                SendCommand("AT+CIPSENDMODE=0\r", true);
                SendEndOfDataCommand();
            }
        }

        // Configure socket parameters
        // AT+CIPCCFG= <NmRetry>, <DelayTm>, <Ack>, <errMode>, <HeaderType>,
        //            <AsyncMode>, <TimeoutVal>
        // NmRetry = number of retransmission to be made for an IP packet
        //         = 10 (default)
        // DelayTm = number of milliseconds to delay before outputting received data
        //          = 0 (default)
        // Ack = sets whether reporting a string "Send ok" = 0 (don't report)
        // errMode = mode of reporting error result code = 0 (numberic values)
        // HeaderType = which data header of receiving data in multi-client mode
        //            = 1 (+RECEIVE,<link num>,<data length>)
        // AsyncMode = sets mode of executing commands
        //           = 0 (synchronous command executing)
        // TimeoutVal = minimum retransmission timeout in milliseconds = 75000
        public void NetworkSetupSocketParamaters()
        {
            Debug.WriteLine("----------------(NetworkSetupSocketParamaters)----------------");

            SendCommand("AT+CIPCCFG=10,0,0,0,1,0,75000\r", true);
            SendEndOfDataCommand();
        }

        // Configure timeouts for opening and closing sockets
        // AT+CIPTIMEOUT=<netopen_timeout> <cipopen_timeout>, <cipsend_timeout>
        public void NetworkSetTCPIP_Timeout()
        {

            Debug.WriteLine("----------------(NetworkSetTCPIP_Timeout)----------------");

            SendCommand("AT+CIPTIMEOUT=75000,15000,15000\r", true);
            SendEndOfDataCommand();
        }

        // Gets the modem's registration status via CREG/CGREG/CEREG
        // CREG = Generic network registration
        // CGREG = GPRS service registration
        // CEREG = EPS registration for LTE modules
        public bool NetworkGetRegistrationState()
        {

            Debug.WriteLine("----------------(NetworkGetRegistrationState)----------------");

            bool isConnected = false;

            SendCommand("AT+CEREG\r", true);
            SendEndOfDataCommand();
            SendCommand("AT+CEREG?\r", true);
            if (_lastResult.IndexOf("1") >= 0)
            {
                Debug.WriteLine("EPS registration for LTE modules detected..");
            }

            SendCommand("AT+CGREG?\r", true);
            SendEndOfDataCommand();
            SendCommand("AT+CGREG?\r", true);
            if (_lastResult.IndexOf("2") >= 0)
            {
                Debug.WriteLine("GPRS service registration detected...");
            }

            SendCommand("AT+CREG\r", true);
            SendEndOfDataCommand();
            SendCommand("AT+CREG?\r", true);
            if (_lastResult.IndexOf("1") >= 0)
            {
                Debug.WriteLine("Generic network registration detected...");
            }

            isConnected = NetworkisConnected();

            return isConnected;
        }

        // Start the socket service
        public void NetworkStartSocketService()
        {
            Debug.WriteLine("----------------(NetworkStartSocketService)----------------");

            SendCommand("AT+NETOPEN\r", true);
            SendEndOfDataCommand();

            if (_lastResult.IndexOf("NETOPEN: 0") >= 0)
            {
                Debug.WriteLine("Detected NETOPEN: 0");
            }
            else if (_lastResult.IndexOf("NETOPEN: 1") >= 0)
            {
                Debug.WriteLine("Detected NETOPEN: 1");
                GetIPaddressOfPDPContext();
            };

            SendCommand("AT+NETOPEN?\r", true);
            if (_lastResult.IndexOf("NETOPEN: 1") >= 0)
            {
                Debug.WriteLine("Detected NETOPEN: 1");
                SendEndOfDataCommand();
            }

            GetIPaddressOfPDPContext();
        }

        public bool NetworkisConnected()
        {
            Debug.WriteLine("----------------(Check if NetworkisConnected)----------------");

            bool isConnected = false;

            SendCommand("AT+NETOPEN?\r", true);
            if (_lastResult.IndexOf("NETOPEN: 1") >= 0)
            {
                Debug.WriteLine("Detected NETOPEN: 1");

                _isConnected = true;
                isConnected = true;

                GetIPaddressOfPDPContext();
            }

            SendEndOfDataCommand();

            return isConnected;
        }

        public void GetIPaddressOfPDPContext()
        {
            Debug.WriteLine("----------------(GetIPaddressOfPDPContext)----------------");

            SendCommand("AT+IPADDR\r", true);
            SendEndOfDataCommand();
        }

        public void ReadICCIDFromSimCard()
        {
            Debug.WriteLine("----------------(ReadICCIDFromSimCard)----------------");

            SendCommand("AT+CICCID\r", true);
            SendEndOfDataCommand();
        }

        public void DumpCMD()
        {
            Debug.WriteLine("----------------(DumpCMD)----------------");

            SendCommand("AT+CMD?\r", true);
            SendEndOfDataCommand();
        }

        public void Firmware()
        {
            Debug.WriteLine("----------------(Firmware)----------------");

            SendCommand("AT+GMR\r", true);
            SendEndOfDataCommand();
        }

        public void ManufacturerInfo()
        {
            Debug.WriteLine("----------------(ManufacturerInfo)----------------");

            SendCommand("AT+CGMI\r", true);
            SendEndOfDataCommand();
        }

        public void About()
        {
            Debug.WriteLine("----------------(About)----------------");

            SendCommand("AT+GMI\r", true);
            SendEndOfDataCommand();

            RequestModelIdentification();
            Firmware();
        }

        public void Location()
        {
            Debug.WriteLine("----------------(Location)----------------");

            SendCommand("AT+CIPGSMLOC=1,1\r", true);
            SendEndOfDataCommand();
        }

        public void Dial(string phoneNumber)
        {
            Debug.WriteLine("----------------(Dial)----------------");

            SendCommand($"ATD{phoneNumber}\r", true);
            SendEndOfDataCommand();
        }

        /// <summary>
        /// Send Message to Number ex: +27824030752
        /// </summary>
        /// <param name="phoneNumber"></param>
        /// <param name="message"></param>
        public void SMS(string phoneNumber, string message)
        {
            Debug.WriteLine("----------------(SMS)----------------");

            SendCommand("AT+CMGF=1\r", true);
            SendCommand($"AT+CMGS=\"{phoneNumber}\"\r", true);
            SendCommand($"{message}\r");
            SendEndOfDataCommand();
        }

        /// <summary>
        ///  Read one SMS examples  SMS with no 1 
        /// </summary>
        /// <param name="msgno"></param>
        public void SMSRead(string msgno)
        {
            SendCommand("AT+CMGF=1\r", true);
            SendCommand("AT+CMGR=" + msgno + "\r", true);
            SendEndOfDataCommand();
        }
        /// <summary>
        /// Read All SMS from SIM 
        /// </summary>
        public void SMSReadAll()
        {
            SendCommand("AT+CMGF=1\r", true);
            SendCommand("AT+CMGL=\"ALL\"\r", true);
            SendEndOfDataCommand();
        }

        public void DelAllSMSRead()
        {
            SendCommand("AT+CMGF=1\r", true);
            SendCommand("AT+CMDA=\"DEL READ\"\r", true);
            SendEndOfDataCommand();
        }

        public void DelAllSMSUnRead()
        {
            SendCommand("AT+CMGF=1\r", true);
            SendCommand("AT+CMDA=\"DEL UNREAD\"\r", true);
            SendEndOfDataCommand();
        }

        public void DelAllSMSSent()
        {
            SendCommand("AT+CMGF=1\r", true);
            SendCommand("AT+CMDA=\"DEL SENT\"\r", true);
            SendEndOfDataCommand();
        }

        public void DelAllSMSUnSent()
        {
            SendCommand("AT+CMGF=1\r", true);
            SendCommand("AT+CMDA=\"DEL UNSENT\"\r", true);
            SendEndOfDataCommand();
        }

        public void DelAllSMSInbox()
        {
            SendCommand("AT+CMGF=1\r", true);
            SendCommand("AT+CMDA=\"DEL INBOX\"\r", true);
            SendEndOfDataCommand();
        }

        public void DelAllSMS()
        {
            SendCommand("AT+CMGF=1\r", true);
            SendCommand("AT+CMDA=\"DEL ALL\"\r", true);
            SendEndOfDataCommand();
        }

        public void http(string url)
        {
            var connectAttempts = 1;
            var errorOccurred = false;

            // _serial.;
            //Debug.WriteLine("Step 1 - HTTP INIT");
            SendCommand("AT+HTTPINIT\r\n", true);

            Debug.WriteLine(_lastResult);

            while (_lastResult.IndexOf("CONNECT OK") < 0 && _lastResult.IndexOf("ALREADY CONNECT") < 0 && connectAttempts <= 3)
            {
                _serialDataFinished.WaitOne(5000, false);
                connectAttempts++;
            }

            if (_lastResult.IndexOf("CONNECT OK") >= 0 || _lastResult.IndexOf("ALREADY CONNECT") >= 0)
            {
                SendCommand("AT+CIPSTATUS\r\n", true);
                Thread.Sleep(1000);

                if (_lastResult.IndexOf("ERROR") > 0)
                {
                    SendCommand("AT+HTTPTERM\r\n", true);

                    HandleFailure();

                    http(url);

                    errorOccurred = true;
                }

                if (!errorOccurred)
                {
                    Debug.WriteLine("Step 2 - ");
                    Thread.Sleep(500);
                    SendCommand("AT+HTTPINIT\r\n", true);
                    Thread.Sleep(400);
                    Debug.WriteLine("HTTP PARA CID");
                    SendCommand("AT+HTTPPARA=\"CID\",1\r\n", true);
                    Thread.Sleep(400);
                    Debug.WriteLine("HTTP URL");
                    SendCommand("AT+HTTPPARA=\"URL\",\"" + url + "\"\r\n", true);
                    //http://exploreembedded.com/wiki/images/1/15/Hello.txt
                    Thread.Sleep(400);
                    Debug.WriteLine("HTTP ACTION");
                    SendCommand("AT+HTTPACTION=0\r\n", true);
                    Thread.Sleep(400);
                    Debug.WriteLine("HTTP READ");
                    SendCommand("AT+HTTPREAD\r\n", true);
                    Thread.Sleep(400);
                    //_serial.Flush();

                    SendEndOfDataCommand();

                    _serialDataFinished.WaitOne(5000, false);

                    SendCommand("AT+HTTPTERM\r\n", true);
                    Thread.Sleep(400);

                    if (_lastResult.IndexOf("ERROR") > 0)
                    {
                        SendCommand("AT+HTTPTERM\r\n", true);

                        HandleFailure();

                        http(url);
                    }
                    else
                    {
                        _failures = 0;
                    }
                }
            }
            else
            {
                Debug.WriteLine("Error on open connection.  Re-initializing.");

                HandleFailure();

                http(url);
            }
        }

        public void HttpGet(string url)
        {
            Debug.WriteLine("----------------(HttpGet)----------------");

            // Set HTTP parameters
            SendCommand("AT+HTTPPARA=\"CID\",1\r", true);
            SendEndOfDataCommand();
            SendCommand($"AT+HTTPPARA=\"URL\",\"{url}\"\r", true);
            SendEndOfDataCommand();

            // Set HTTP action
            SendCommand("AT+HTTPACTION=0\r", true);
            SendEndOfDataCommand();

            // Read HTTP response
            SendCommand("AT+HTTPREAD\r", true);
            SendEndOfDataCommand();
        }

        public void Get(string host, string page, string contentType, string data)
        {
            var connectAttempts = 1;
            var errorOccurred = false;

            // _serial.;

            SendCommand("AT+CIPSTART=\"TCP\",\"" + host + "\",\"\"\r", true);

            Debug.WriteLine(_lastResult);

            while (_lastResult.IndexOf("CONNECT OK") < 0 && _lastResult.IndexOf("ALREADY CONNECT") < 0 && connectAttempts <= 3)
            {
                _serialDataFinished.WaitOne(5000, false);
                connectAttempts++;
            }

            if (_lastResult.IndexOf("CONNECT OK") >= 0 || _lastResult.IndexOf("ALREADY CONNECT") >= 0)
            {
                SendCommand("AT+CIPSTATUS\r", true);
                Thread.Sleep(1000);
                SendCommand("AT+CIPSEND\r", true);

                if (_lastResult.IndexOf("ERROR") > 0)
                {
                    HandleFailure();

                    Get(host, page, contentType, data);

                    errorOccurred = true;
                }

                if (!errorOccurred)
                {
                    Thread.Sleep(500);

                    SendCommand("GET " + page + " HTTP/1.1\r\n");
                    SendCommand("Host: " + host + "\r\n");
                    SendCommand("Content-Length: " + data.Length + "\r\n");
                    SendCommand("Content-Type: " + contentType + "\r\n\r\n");
                    SendCommand(data + "\r");

                    //_serial.Flush();

                    SendEndOfDataCommand();

                    _serialDataFinished.WaitOne(5000, false);

                    SendCommand("AT+CIPCLOSE\r", true);

                    if (_lastResult.IndexOf("ERROR") > 0)
                    {
                        HandleFailure();

                        Get(host, page, contentType, data);
                    }
                    else
                    {
                        _failures = 0;
                    }
                }
            }
            else
            {
                Debug.WriteLine("Error on open connection.  Re-initializing.");

                HandleFailure();

                Get(host, page, contentType, data);
            }
        }

        public string Post(string host, string page, string contentType, string data)
        {
            var connectAttempts = 1;
            var errorOccurred = false;
            var jsontoken = "";

            // Close previous HTTP connection
            SendCommand("AT+HTTPTERM\r", true);
            _serialDataFinished.WaitOne(1000, false);

            // Close HTTP connection
            SendCommand("AT+HTTPINIT\r", true);
            SendCommand("AT+HTTPPARA=\"CID\",1\r", true);
            SendCommand($"AT+HTTPPARA=\"URL\",\"{host}{page}\"\r", true);
            SendCommand($"AT+HTTPPARA=\"CONTENT\",\"{contentType}\"\r", true);

            // Add the Authorization header if the auth token has been previously set
            if (!string.IsNullOrEmpty(_authToken))
            {
                SendCommand("AT+HTTPPARA=\"USERDATA\",\"Authorization: Bearer " + _authToken + "\"\r\n");
            }

            // Prepare data
            var dataToSend = Encoding.UTF8.GetBytes(data);
            SendCommand($"AT+HTTPDATA={dataToSend.Length},10000\r", true);
            _serialDataFinished.WaitOne(1000, false);

            if (_lastResult.IndexOf("DOWNLOAD") >= 0)
            {
                _serial.Write(dataToSend, 0, dataToSend.Length);
                _serialDataFinished.WaitOne(1000, false);
            }
            else
            {
                errorOccurred = true;
            }

            if (!errorOccurred)
            {
                // Perform POST request
                SendCommand("AT+HTTPACTION=1\r", true);
                _serialDataFinished.WaitOne(2000, false);

                if (_lastResult.IndexOf("HTTPACTION: 1") >= 0)
                {
                    // AT+HTTPHEAD Read the HTTP(S) Header Information of Server Response
                    SendCommand("AT+HTTPHEAD\r", true);

                    // Get data length
                    SendCommand("AT+HTTPREAD?\r", true, 1000);

                    // Get the len int value
                    int index = _lastResult.IndexOf("LEN,") + 4;
                    int endIndex = _lastResult.IndexOf("\r", index);
                    string lenStr = _lastResult.Substring(index, endIndex - index);
                    int length;

                    if (int.TryParse(lenStr, out length))
                    {
                        // Read response data using the length
                        SendCommand($"AT+HTTPREAD=0,{length}\r", true, 5000);
                    }

                    // Retrieve the token val if not already set
                    if (string.IsNullOrEmpty(_authToken))
                    {
                        // Find the position of the first occurrence of the "token" substring
                        int start = _lastResult.IndexOf("\"token\":");

                        // If "token" is found, find the position of the first double quote after it
                        if (start >= 0)
                        {
                            int end = _lastResult.IndexOf('"', start + 9);

                            // If the ending double quote is found, extract the substring between the two double quotes
                            if (end > start)
                            {
                                string token = _lastResult.Substring(start + 9, end - start - 9);
                                _authToken = token;
                                Debug.WriteLine($"We got Token: {token}");
                            }
                        }
                    }

                    // TODO: Better error handling, this is old and not very effective.
                    if (_lastResult.IndexOf("ERROR") > 0)
                    {
                        HandleFailure();
                        Get(host, page, contentType, data);
                    }

                    // Close HTTP connection
                    SendCommand("AT+HTTPTERM\r", true);
                    _serialDataFinished.WaitOne(1000, false);
                }
                else
                {
                    HandleFailure();
                    Post(host, page, contentType, data);
                }
            }
            else
            {
                Debug.WriteLine("Error on open connection. Re-initializing.");
                HandleFailure();
                Post(host, page, contentType, data);
            }

            return _lastResult.ToString();
        }

        public void GetAuthToken(string host, string endpoint, string username, string password)
        {
            // Create jsondata
            string postData = "{\"username\": \"" + username + "\", \"password\": \"" + password + "\"}";

            // Post
            Post(host, endpoint, "application/json", postData);
        }

        private void HandleFailure()
        {
            _failures++;

            Debug.WriteLine($"Failures: {_failures}");

            // Close the connection or shut it down depending on the number of failures
            if (_failures % 2 == 0)
            {
                SendCommand("AT+CIPSHUT\r", true);
                InitializeModem();
            }
            else
            {
                SendCommand("AT+CIPCLOSE\r", true);
            }

            // Sleep for a second allowing events to process to hopefully sync up the serial data communication
            Thread.Sleep(1000);
        }

        public void SetSSLContext()
        {
            Debug.WriteLine("----------------(SetSSLContext)----------------");

            SendCommand("AT+CSSLCFG=\"sslversion\",3,4\r", true);
            SendEndOfDataCommand();

            SendCommand("AT+CSSLCFG=\"ciphersuite\",0xFFFF\r", true);
            SendEndOfDataCommand();

            SendCommand("AT+CSSLCFG=\"seclevel\",0\r", true);
            SendEndOfDataCommand();

            SendCommand("AT+CSSLCFG=\"cacert\",0\r", true);
            SendEndOfDataCommand();

            SendCommand("AT+CSSLCFG=\"clientcert\",0\r", true);
            SendEndOfDataCommand();

            SendCommand("AT+CSSLCFG=\"clientkey\",0\r", true);
            SendEndOfDataCommand();
        }

        public void StartSSLService()
        {
            Debug.WriteLine("----------------(StartSSLService)----------------");

            SendCommand("AT+CCHSTART\r", true);
            SendEndOfDataCommand();
        }

        public void StopSSLService()
        {
            Debug.WriteLine("----------------(StopSSLService)----------------");

            SendCommand("AT+CCHSTOP\r", true);
            SendEndOfDataCommand();
        }

        public void ConnectToSSLServer(string host, int port)
        {
            Debug.WriteLine("----------------(ConnectToSSLServer)----------------");

            SendCommand($"AT+CCHOPEN=\"SSL\",\"{host}\",\"{port}\"\r", true);
            SendEndOfDataCommand();
        }

        public void SendDataToSSLServer(string data)
        {
            Debug.WriteLine("----------------(SendDataToSSLServer)----------------");

            var dataToSend = Encoding.UTF8.GetBytes(data);
            SendCommand($"AT+CCHSEND={dataToSend.Length}\r", true);
            SendCommand(data, false);
            SendEndOfDataCommand();
        }

        public string ReadDataFromSSLServer()
        {
            Debug.WriteLine("----------------(ReadDataFromSSLServer)----------------");

            SendCommand("AT+CCHRECV\r", true);
            SendEndOfDataCommand();

            return _lastResult;
        }

        public void ConfigureSSLClientContext()
        {
            Debug.WriteLine("----------------(ConfigureSSLClientContext)----------------");

            SendCommand("AT+CCHCFG=\"sslctxid\",1\r", true);
            SendEndOfDataCommand();

            SendCommand("AT+CCHCFG=\"sslversion\",3\r", true);
            SendEndOfDataCommand();

            SendCommand("AT+CCHCFG=\"ciphersuite\",0xFFFF\r", true);
            SendEndOfDataCommand();

            SendCommand("AT+CCHCFG=\"seclevel\",0\r", true);
            SendEndOfDataCommand();
        }

        private void GpsDataReceived(object sender, SerialDataReceivedEventArgs e)
        {
            if (_serial.BytesToRead == 0)
            {
                return;
            }

            byte[] buffer = new byte[_serial.BytesToRead];
            int bytesRead = _serial.Read(buffer, 0, buffer.Length);

            for (int i = 0; i < bytesRead; i++)
            {
                if (s_gps.Encode((char)buffer[i]))
                {
                    GetGpsData();
                }
            }
        }

        private static string GetGpsData()
        {
            StringBuilder sb = new StringBuilder();

            sb.Append("Location: ");
            if (s_gps.Location.IsValid)
            {
                sb.Append(s_gps.Location.Latitude.Degrees.ToString());
                sb.Append(",");
                sb.Append(s_gps.Location.Longitude.Degrees.ToString());
            }
            else
            {
                sb.Append("INVALID");
            }

            sb.Append(" Date/Time: ");
            if (s_gps.Date.IsValid)
            {
                sb.Append(s_gps.Date.Year.ToString());
                sb.Append("/");
                sb.Append(s_gps.Date.Month.ToString("D2"));
                sb.Append("/");
                sb.Append(s_gps.Date.Day.ToString("D2"));
            }
            else
            {
                sb.Append("INVALID");
            }

            if (s_gps.Time.IsValid)
            {
                sb.Append(" ");
                sb.Append(s_gps.Time.Hour.ToString("D2"));
                sb.Append(":");
                sb.Append(s_gps.Time.Minute.ToString("D2"));
                sb.Append(":");
                sb.Append(s_gps.Time.Second.ToString("D2"));
                sb.Append(".");
                sb.Append(s_gps.Time.Centisecond.ToString("D2"));
            }
            else
            {
                sb.Append(" INVALID");
            }

            return sb.ToString();
        }

    }
}