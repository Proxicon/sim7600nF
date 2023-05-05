using System;
using System.Diagnostics;
using nanoFramework.Hardware.Esp32;
using sim7600x;
using System.Device.Adc;

namespace Sim7600_Test
{
    public class Program
    {
        // Modem specific
        private static int LED = 12;
        private static int MODEM_TX = 27;
        private static int MODEM_RX = 26;
        private static int MODEM_PWRKEY = 4;
        // private static int MODEM_DTR = 32;
        // private static int MODEM_RI = 33;
        private static int MODEM_FLIGHT = 25;
        // private static int MODEM_STATUS = 34;

        // APN details
        private static string APN = "myMTN";
        private static string APNUser = "";
        private static string APNPass = "";

        // Battery monitoring
        // Create an ADC object to read the voltage ADC1 channel 1 is pin 35 on ESP32
        // private static AdcController adc1;

        private static AdcController adc1 = new AdcController();
        private static AdcChannel channel0 = adc1.OpenChannel(0);

        public static void Main()
        {

            Debug.WriteLine($"Configure esp32 serial pins: TX({MODEM_TX}) RX:({MODEM_RX})");
            Debug.WriteLine("------------------------------");
            Configuration.SetPinFunction(MODEM_RX, DeviceFunction.COM2_RX);
            Configuration.SetPinFunction(MODEM_TX, DeviceFunction.COM2_TX);

            Debug.WriteLine("Init Sim - Will power on chip,gps & connect to network");
            Debug.WriteLine("------------------------------");

            PrintMemoryInfo();

            // Init modem
            var sim = new sim7600(APN, APNUser, APNPass, "COM2", MODEM_PWRKEY, MODEM_FLIGHT, LED);

            sim.InitializeModem();

            // if AT response fails 5x, sim chip will be restarted & retried
            Debug.WriteLine("At Commands - restarts chip if no response x5");
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
                sim.OperatorSelection();

                //sim.NetworkSetAuthType(APNUser, APNPass);
                //sim.TestPDPContext();

            }
            else
            {
                Debug.WriteLine("NetworkisConnected() == false, calling InitializeModem()");
                sim.InitializeModem();
                sim.OperatorSelection();
            }

            // Location - Set GPS config
            sim.ConfigureGNSSSupportMode();

            sim.ReadICCIDFromSimCard();

            // Start GPS Session
            sim.StartStopGpsSession(1);

            Debug.WriteLine("Starting main loop next... HTTP post tests.");

            // start main loop
            while (true)
            {
                Debug.WriteLine("Calling: sim.GetGPSFixedPositionInformation(); sleep 3000");
                string gpsData = sim.GetGPSFixedPositionInformation();

                try
                {
                    PrintMemoryInfo();

                    // Check if the sim context have an active token.
                    string authToken = sim.AuthToken;

                    if (string.IsNullOrEmpty(authToken))
                    {
                        // Get initial auth token
                        Debug.WriteLine("Calling: sim.GetAuthToken(\"sim.proxicon.co.za\", \"/token\", \"admin\", \"admin\")");
                        sim.GetAuthToken("http://sim.proxicon.co.za", "/token", "admin", "admin");

                    }
                    else
                    {
                        // Use the authToken & post GPS data
                        Debug.WriteLine("Calling: sim.Post(\"sim.proxicon.co.za\", \"/simlogs\", \"application/json\", simlogs);");

                        string simlogs = $"{{\"id\":0,\"device\":\"Esp32DEV00\",\"logitem\":\"GPSData\",\"message\":\"{gpsData}\"}}";
                        Debug.WriteLine($"simlogs: {simlogs}");

                        // Exclude empty datasets
                        if (!string.IsNullOrEmpty(gpsData) && gpsData != ",,,,,,,,")
                        {
                            sim.Post("http://sim.proxicon.co.za", "/simlogs", "application/json", simlogs);
                        }

                        // Monitor battery %
                        //int batteryPercentage = GetBatteryPercentage();
                        //Debug.WriteLine($"Battery percentage: {batteryPercentage}%");

                    }
                }
                catch (Exception ex)
                {
                    Debug.WriteLine("Error posting GPS data: " + ex.Message);

                    Debug.WriteLine("Closing HTTP Connection..");

                    // Close previous HTTP connection on failures
                    sim.SendCommand("AT+HTTPTERM\r", true);

                    // Null Auth Token
                    sim.AuthToken = null;

                }
            }
        }

        public static void PrintMemoryInfo()
        {
            NativeMemory.GetMemoryInfo(
                NativeMemory.MemoryType.Internal,
                out var totalSize,
                out var freeSize,
                out var largestFreeBlock);

            var usedPercentage = (totalSize - freeSize) * 100 / totalSize;
            Debug.WriteLine($"Total memory: {totalSize}, Free memory: {freeSize}, Largest free block: {largestFreeBlock}, Used percentage: {usedPercentage}%");
        }

        public int GetBatteryPercentage()
        {
            // Read the raw voltage value from the ADC channel
            int rawVoltage = channel0.ReadValue();

            // Get the maximum and minimum raw values from the ADC controller
            int maxRawValue = adc1.MaxValue;
            int minRawValue = adc1.MinValue;

            // Battery % Calculation
            // Convert the raw voltage value to a voltage level in volts
            double voltage = (rawVoltage * 3.3) / maxRawValue;

            // Calculate the battery percentage based on the voltage level
            int batteryPercentage = (int)((voltage - 3.0) / (4.2 - 3.0) * 100);

            return batteryPercentage;
        }
    }
}