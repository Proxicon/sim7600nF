using System;
using System.Diagnostics;
using System.Threading;

using nanoFramework.Hardware.Esp32;
using System.Device.Gpio;
using System.IO.Ports;
using sim7600x;

namespace Sim7600_Test
{
    public class Program
    {
        static SerialPort port = null;

        // Modem specific
        private static int LED = 12;
        private static int MODEM_TX = 27;
        private static int MODEM_RX = 26;
        private static int MODEM_PWRKEY = 4;
        private static int MODEM_DTR = 32;
        private static int MODEM_RI = 33;
        private static int MODEM_FLIGHT = 25;
        private static int MODEM_STATUS = 34;

        /* APM Specific
            Source1: https://sabroadband.co.za/mtn-lte-apn-settings/
            Name: MTN Internet
            APN: internet
            Username: guest
            Password:  Not Set

            Source2: 
            APN: internet 
            Username: guest 
            Password: ( ) 
            Read more: https://briefly.co.za/43137-mtn-apn-settings-south-africa-internet-settings-apn-settings-south-africa.html

         */
        private static string APN = "internet";
        private static string APNUser = "guest";
        private static string APNPass = "";

        public static void Main()
        {

            Debug.WriteLine($"Configure esp32 serial pins: TX({MODEM_TX}) RX:({MODEM_RX})");
            Debug.WriteLine("------------------------------");
            Configuration.SetPinFunction(MODEM_RX, DeviceFunction.COM2_RX);
            Configuration.SetPinFunction(MODEM_TX, DeviceFunction.COM2_TX);

            Debug.WriteLine("Init Sim - Will ");
            Debug.WriteLine("------------------------------");
            // APN Details
            /*      private const string APN = "internet";
                    private const string gprsUser = "guest";
                    private const string gprsPass = "";
            */

            // Init modem
            var sim = new sim7600(APN, "COM2", MODEM_PWRKEY, MODEM_FLIGHT, LED);

            // if AT response fails 5x, sim chip will be restarted & retried
            Debug.WriteLine("Do Simple Sim Check At Commands");
            Debug.WriteLine("------------------------------");
            sim.SimCheck();

            Debug.WriteLine("About SIM Device");
            Debug.WriteLine("------------------------------");
            sim.About();
            Debug.WriteLine("------------------------------");
            Debug.WriteLine("End of test");

            // Network Connection Related
            Debug.WriteLine("InitializeModem");
            Debug.WriteLine("------------------------------");

            // Start modem connection sequence if its not already connected
            if (sim.NetworkisConnected() == true)
            {
                Debug.WriteLine("Bypassing InitializeModem(), NetworkisConnected() == true");
            }
            else
            {
                sim.InitializeModem();
            }

            // Location - Set GPS config
            sim.ConfigureGNSSSupportMode();
            
            // Url stuff
            //sim.ipko("http://exploreembedded.com/wiki/images/1/15/Hello.txt");
            //sim.Get("exploreembedded.com", 80, "/wiki/images/1/15/Hello.txt", "application/x-www-form-urlencoded", "");
            //sim.Get("baseurl.com", 80, "/somePathOnThatUrl", "application/x-www-form-urlencoded", "{\"Key\":\"Value\"}"); //is not finished yet ...
            //sim.Post("baseurl.com", 80, "/somePathOnThatUrl", "application/json", "{\"Key\":\"Value\"}");

            sim.ReadICCIDFromSimCard();

            //Debug.WriteLine("Test 1");
            Thread.Sleep(1000);
            sim.SMS("+27824030752", "ESP32 Sim7600x - Nanoframework SMS Yo!");
            //Debug.WriteLine("Test 2");
            //sim.SMS("044173830", "Tung from Sim7600xL test 2");
            //Thread.Sleep(1000);

            // Debug.WriteLine("Test 3");
            // sim.SMSReadAll();
            // Thread.Sleep(1000);

            // Start GPS Session
            sim.StartStopGpsSession(1);

            // 5000ms pause

            sim.GetGPSFixedPositionInformation();

            while (true)
            {
                sim.GetGPSFixedPositionInformation();
                Thread.Sleep(3000);
            }
        }

        private static void Port_DataReceived(object sender, SerialDataReceivedEventArgs e)
        {
            SerialPort recivedinfo = (SerialPort)sender;
            int byteforreads = recivedinfo.BytesToRead;
            byte[] buffer = new byte[byteforreads];

            recivedinfo.Read(buffer, 0, buffer.Length);
            Debug.WriteLine("Received data : ");
            Debug.WriteLine(System.Text.Encoding.UTF8.GetString(buffer, 0, buffer.Length));
        }

        private static void UartSend(string toSend)
        {
            byte[] bytesToSent = System.Text.Encoding.UTF8.GetBytes(toSend + "\r\n");
            port.Write(toSend);
        }
    }
}
