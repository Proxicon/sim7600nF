using System;
using System.IO.Ports;
using System.Text;
using System.Diagnostics;
using System.Device.Gpio;
using System.Threading;
using nanoFramework.Runtime.Events;
using TinyGPSPlusNF;
using System.Text.RegularExpressions;

namespace sim7600x
{
    public class sim7600
    {
        private readonly SerialPort _serial;
        private readonly StringBuilder _resultBuffer = new StringBuilder();
        private readonly AutoResetEvent _serialDataFinished = new AutoResetEvent(false);
        private string _lastResult = "";
        private readonly string _apn;
        private int _failures = 0;
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

        // public int pwkkey;
        // public int rstkey;
        // public int poweronkey;
        public sim7600(string apn, string portName, int pwkkey, int flight, int ledpin)
        {
            _apn = apn;

            // init led & turn it off
            _ledPower = new GpioController().OpenPin(ledpin, PinMode.Output);
            _ledPower.Write(PinValue.Low);

            // set modem power
            _MODEM_PWRKEY = new GpioController().OpenPin(pwkkey);
            _MODEM_PWRKEY.SetPinMode(PinMode.Output);

            // Enable GPS for recieving AT commands

            /*For SIM7600E-H-M2/SIM7600SA-H-M2/SIM7600A-H-M2 module, GPS started should be decided
            by the physical switch of GPS flight mode in the module firstly. Open the switch, GPS will be
            started automatically, then you can open or close gps by AT command, otherwize, GPS could not
            be started in any way.it will report +CME ERROR:GPS flight mode enabl
            */

            if (flight > 0)
            {
                _MODEM_FLIGHT = new GpioController().OpenPin(flight);
                _MODEM_FLIGHT.SetPinMode(PinMode.Output);
                _MODEM_FLIGHT.Write(PinValue.High);
            }


            // create TinyGPS
            TinyGPSPlus gps = new();

            _serial = new SerialPort(portName, 115200, Parity.None, 8, StopBits.One);
            _serial.Handshake = Handshake.RequestToSend;

            _serial.DataReceived += SerialOnDataReceived;

            _serial.Open();

            /*
            Turns on the chip, SIM chip stays online after flashing new firmware delaying the startup sequence
            we add a check first to see if the network is connected before turning it off and back on
            */

            // check automatice time & timezome update setting
            AutomaticTimeandTimezoneUpdate();

            // check connectivity state
            bool isConnected = NetworkisConnected();

            if (isConnected)
            {
                Debug.WriteLine("Modem is connected to network, no need to cycle power...\n");

                /* indicate modem can be controlled*/
                _ledPower.Toggle();
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
                }
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

                    _resultBuffer.Clear();

                    Debug.WriteLine("SerialOnDataReceived._lastResult" + _lastResult);

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
                NetworkStopTCPIPService();

                // Set APN details 
                NetworkDefinePDPConext("internet");
                NetworkSetAuthType("guest",0);

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

        public void SendCommand(string command, bool waitForResponse = false, int timeout = 1000)
        {
            Debug.WriteLine($"AT+Comm = [{command}]\r");

            _serialDataFinished.Reset();

            _lastResult = "";

            var writeBuffer = Encoding.UTF8.GetBytes(command);

            _serial.Write(writeBuffer, 0, writeBuffer.Length);

            if (waitForResponse)
            {
                _serialDataFinished.WaitOne(timeout, false);
            }
        }

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

        /*
        public string GetGPSFixedPositionInformation()
        {
            Debug.WriteLine("----------------(GetGPSFixedPositionInformation)----------------");

            SendCommand("AT+CGPSINFO\r", true);
            SendEndOfDataCommand();

            string response = _lastResult;
            Debug.WriteLine("The _lastResult response was: " + response);

            // Extract GPS data from the response
            var match = Regex.Match(response, @"\+CGPSINFO: (.*)");
            if (match.Success)
            {
                string gpsInfo = match.Groups[1].Value;
                return gpsInfo;
            }
            else
            {
                Debug.WriteLine("Failed to retrieve GPS information.");
                return null;
            }
        }
        */

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

            GetIPaddressOfPDPContext();
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
            SendEndOfDataCommand();

            // Write Command
            // 0 auto, 2 GSM only, 38 LTE
            SendCommand("AT+CNMP=0\r", true);
            SendEndOfDataCommand();
        }

        public void NetworkStopTCPIPService()
        {
            Debug.WriteLine("----------------(NetworkStopTCPIPService)----------------");

            /*SendCommand("AT+CIPCLOSE\r", true);
            SendEndOfDataCommand();*/

            SendCommand("AT+NETCLOSE\r", true);
            SendEndOfDataCommand();
        }

        public void NetworkSetAuthType(string User = "", int Password = 0)
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
            SendCommand($"AT+CGDCONT=1,\"IP\",\"{APN}\",\"0.0.0.0\",0,0\r", true);
            SendEndOfDataCommand();

            // Execute command
            SendCommand("AT+CGDCONT\r", true);
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

        public void ipko(string url)
        {
            SendCommand("AT+CREG?\r\n", true);
            SendEndOfDataCommand();

            SendCommand("AT+SAPBR=3,1,\"Contype\",\"GPRS\"\r\n", true);
            SendEndOfDataCommand();
            Thread.Sleep(100);
            SendCommand("AT+SAPBR=3,1,\"APN\",\"IPKO\"\r\n", true);
            SendEndOfDataCommand();
            Thread.Sleep(100);
            SendCommand("AT+SAPBR=3,1,\"USER\",\"\"\r\n", true);
            SendEndOfDataCommand();
            Thread.Sleep(100);
            SendCommand("AT+SAPBR=1,1\r\n", true);
            SendEndOfDataCommand();
            Thread.Sleep(100);
            SendCommand("AT+SAPBR=2,1\r\n", true);
            SendEndOfDataCommand();
            Thread.Sleep(100);
            SendCommand("AT+CLBS=1,1\r\n", true);
            SendEndOfDataCommand();
            Thread.Sleep(100);
            SendCommand("AT+CLTS?\r\n", true);
            SendEndOfDataCommand();
            Thread.Sleep(10000);
            SendCommand("AT+CIPGSMLOC=1,1\r\n", true);
            SendEndOfDataCommand();
            Thread.Sleep(15000);
            SendCommand("AT+CIPGSMLOC=2,1\r\n", true);
            SendEndOfDataCommand();
            Thread.Sleep(500);
            SendCommand("AT+CIPGSMLOC=1,1\r\n", true);
            SendEndOfDataCommand();
            Thread.Sleep(15000);
            //SendCommand("AT+SAPBR=0,1\r\n", true);
            //SendEndOfDataCommand();
            //Thread.Sleep(100);
            SendCommand("AT+HTTPINIT\r\n", true);
            SendEndOfDataCommand();
            Thread.Sleep(100);
            // SendCommand("AT+HTTPPARA=\"PROIP\",\"213.229.249.40\"\r\n", true);
            // SendEndOfDataCommand();
            // Thread.Sleep(100);
            // SendCommand("AT+HTTPPARA=\"PROPORT\",\"8080\"\r\n", true);
            // SendEndOfDataCommand();
            // Thread.Sleep(100);
            //SendCommand("AT+HTTPSSL=1\r\n", true);
            //SendEndOfDataCommand();
            //Thread.Sleep(100);
            SendCommand("AT+HTTPPARA=\"CID\",1\r\n", true);
            SendEndOfDataCommand();
            Thread.Sleep(100);
            SendCommand("AT+HTTPPARA=\"URL\",\"" + url + "\"\r\n", true);
            SendEndOfDataCommand();
            Thread.Sleep(100);
            SendCommand("AT+HTTPACTION=0\r\n", true);
            SendEndOfDataCommand();
            Thread.Sleep(10000);
            //SendCommand("AT+HTTPREAD=0,28\r\n", true);
            SendCommand("AT+HTTPREAD\r\n", true);
            SendEndOfDataCommand();

            //Thread.Sleep(100);
            //SendEndOfDataCommand();
            //SendCommand("AT+HTTPTERM\r\n", true);
            //Thread.Sleep(100);
            //https://www.ipko.com/en/faq/si-te-konfiguroj-roaming-ne-telefon/

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

        public void Get(string host, int port, string page, string contentType, string data)
        {
            var connectAttempts = 1;
            var errorOccurred = false;

            // _serial.;

            SendCommand("AT+CIPSTART=\"TCP\",\"" + host + "\",\"" + port + "\"\r", true);

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

                    Get(host, port, page, contentType, data);

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

                        Get(host, port, page, contentType, data);
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

                Get(host, port, page, contentType, data);
            }
        }

        public string Post(string host, int port, string page, string contentType, string data, string authToken = null)
        {
            var connectAttempts = 1;
            var errorOccurred = false;

            // Configure HTTP(s) parameters
            if(port == 443)
            {
                SendCommand("AT+CCHSTART\r", true); //SSL service
                SendCommand("AT+CCHCFG=\"sendtimeout\",\"0\",\"60\"\r", true); //SSL client

                SendCommand("AT+CCERTDOWN=\"client_key.der\",\"611\"\r", true); //Download client

                SendCommand($"AT+CCHOPEN=\"0\",\"{host}{page}\",\"{port}\",\"2\"\r", true); //cost connection
            }

            SendCommand("AT+HTTPINIT\r", true);
            SendCommand("AT+HTTPPARA=\"CID\",1\r", true);
            SendCommand($"AT+HTTPPARA=\"URL\",\"http://{host}:{port}{page}\"\r", true);
            SendCommand($"AT+HTTPPARA=\"CONTENT\",\"{contentType}\"\r", true);
            

            // Add this line to include the Authorization header if authToken is provided
            if (!string.IsNullOrEmpty(authToken))
            {
                SendCommand("AT+HTTPPARA=\"USERDATA\",\"Authorization: Bearer " + authToken + "\"\r\n");
            }

            // Prepare data
            var dataToSend = Encoding.UTF8.GetBytes(data);
            SendCommand($"AT+HTTPDATA={dataToSend.Length},10000\r", true);
            _serialDataFinished.WaitOne(10000, false);

            if (_lastResult.IndexOf("DOWNLOAD") >= 0)
            {
                _serial.Write(dataToSend, 0, dataToSend.Length);
                _serialDataFinished.WaitOne(10000, false);
            }
            else
            {
                errorOccurred = true;
            }

            if (!errorOccurred)
            {
                // Perform POST request
                SendCommand("AT+HTTPACTION=1\r", true);
                _serialDataFinished.WaitOne(120000, false);

                if (_lastResult.IndexOf("HTTPACTION: 1") >= 0)
                {
                    // AT+HTTPHEAD Read the HTTP(S) Header Information of Server Response
                    SendCommand("AT+HTTPHEAD\r", true);

                    // Read response data
                    SendCommand("AT+HTTPREAD\r", true);
                    _serialDataFinished.WaitOne(10000, false);

                    if (_lastResult.IndexOf("ERROR") > 0)
                    {
                        HandleFailure();

                        Get(host, port, page, contentType, data);
                    }

                    // Close HTTP connection
                    SendCommand("AT+HTTPTERM\r", true);
                    _serialDataFinished.WaitOne(1000, false);
                }
                else
                {
                    HandleFailure();
                    Post(host, port, page, contentType, data);
                }
            }
            else
            {
                Debug.WriteLine("Error on open connection. Re-initializing.");
                HandleFailure();
                Post(host, port, page, contentType, data);
            }

            return _lastResult;
        }

        public string GetAuthToken(string host, int port, string endpoint, string username, string password)
        {
            string token = string.Empty;

            string postData = "{\"username\": \"" + username + "\", \"password\": \"" + password + "\"}";

            // Replace the existing Post method call with the modified version
            string response = Post(host, port, endpoint, "application/json", postData);

            // Use a simple JSON parser or regex to extract the token from the response
            Match match = Regex.Match(response, "\\\"token\\\":\\s*\\\"([^\"]+)\\\"");
            if (match.Success)
            {
                token = match.Groups[1].Value;
            }

            return token;
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