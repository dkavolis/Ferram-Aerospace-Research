using System.Collections.Generic;

namespace FerramAerospaceResearch.Utils
{
    public class Metrics
    {
        public Dictionary<string, MetricsElement> data = new Dictionary<string, MetricsElement>();
        const int hysteresisFactor = 20;
        public Metrics() { }
        public void Reset() => data.Clear();
        public void AddMeasurement(string name, double t)
        {
            if (!data.ContainsKey(name))
            {
                data.Add(name, new MetricsElement());
            }
            MetricsElement m = data[name];
            m.iterations++;
            m.hysteresisTime = (m.hysteresisTime * (hysteresisFactor - 1) + t) / hysteresisFactor;
        }
    }

    public class MetricsElement
    {
        public int iterations = 0;
        public double hysteresisTime = 0;

        public MetricsElement() { }
        public override string ToString() => $"iter: {iterations} TimePerRun: {hysteresisTime:F2} ms";
    }
}
