using System; // Recommended to be explicit

// Assuming SensorValue, SensorType, PumpSensorValues are in the same namespace or referenced
// namespace HealthMonitor; // If using a namespace

class Program
{
    static void Main(string[] args)
    {
        Console.WriteLine("Instantierea clasei SensorValue\n\r");
        // instantiere folosind constructorul implicit. Valorile de masurare se dau folosind proprietatile
        SensorValue sensor1 = new SensorValue();
        sensor1.Type = SensorType.BloodGlucose;
        sensor1.TimeStamp = DateTime.Now;
        sensor1.Value = 100; //mg/dl
        DisplaySensorValues("Primul senzor initializat ",sensor1);

        // instantiere folosind constructorul cu parametrii - Corrected date string format
        SensorValue sensor2 = new SensorValue(SensorType.SkinTemperature, 36.7, "23-Jan-10 14:56:00"); // Added :00 for seconds
        DisplaySensorValues("Al doilea senzor initializat ", sensor2);

        // Assuming PumpSensorValues class is updated as below
        PumpSensorValues sensorValuesPump = new PumpSensorValues(3);
        sensorValuesPump.StartPumping();
        Console.WriteLine("Pump running... Press Enter to stop.");
        Console.ReadLine();
        sensorValuesPump.StopPumping();
        Console.WriteLine("Pump stopped.");
    }

    internal static void DisplaySensorValues(string headerText, SensorValue sensor)
    {
        Console.WriteLine("\t " + headerText);
        Console.WriteLine("\t\t Type = {0} ", sensor.Type.ToString());
        Console.WriteLine("\t\t TimeStamp = {0} ", sensor.TimeStampString);
        Console.WriteLine("\t\t Value = {0} ", sensor.Value.ToString("0.00"));
    }
}