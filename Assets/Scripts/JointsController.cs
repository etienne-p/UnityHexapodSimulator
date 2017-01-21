using UnityEngine;
using System.Collections;

[RequireComponent(typeof(CPG))]
public class JointsController : MonoBehaviour
{
    const int N_LEGS = 6;

    public HingeJoint[] shoulderJoints;
    public HingeJoint[] kneeJoints;

    [Range(0, 10)]
    public float sig = 1.0f;
    [Range(-1, 1)]
    public float right = .0f;
    [Range(-1, 1)]
    public float left = .0f;
    [Range(0, 2)]
    public float smoothing = 1.0f;

    CPG cpg;
    // Knee joint phase offset
    float[] phaseOffset;
    // Knee joint phase offset derivative
    float[] dPhaseOffset;
    // Target knee joint phase offset derivative
    float[] tPhaseOffset;
    // Shoulder joint amp
    float[] amp;
    // Shoulder joint amp derivative
    float[] dAmp;
    // Target shoulder joint amp
    float[] tAmp;

    readonly float[] PHASE_OFFSET = new float[]
    {
        0, Mathf.PI, 0, Mathf.PI, 0, Mathf.PI, 0
    };

    void OnGUI()
    {
        GUILayout.BeginArea(new Rect(24, 24, 320, 500));
        GUILayout.Label("Skid steering controls:");
        GUILayout.Label("right");
        right = GUILayout.HorizontalSlider(right, -1, 1);
        GUILayout.Label("left");
        left = GUILayout.HorizontalSlider(left, -1, 1);
        GUILayout.EndArea();
    }

    void OnEnable()
    {
        cpg = GetComponent<CPG>();
        phaseOffset = new float[N_LEGS];
        dPhaseOffset = new float[N_LEGS];
        tPhaseOffset = new float[N_LEGS];
        GetKneeJointPhaseOffset(left, right, ref phaseOffset);
        GetKneeJointPhaseOffset(left, right, ref tPhaseOffset);
        amp = new float[N_LEGS];
        dAmp = new float[N_LEGS];
        tAmp = new float[N_LEGS];
        GetShoulderJointAmp(left, right, ref amp);
        GetShoulderJointAmp(left, right, ref tAmp);
    }

    void Update()
    {
        GetKneeJointPhaseOffset(left, right, ref tPhaseOffset);
        GetShoulderJointAmp(left, right, ref tAmp);

        float dt = Time.deltaTime;
        int i = 0;
        for (; i < N_LEGS; ++i)
        {
            // Critically damped second order differential equation
            // http://mathproofs.blogspot.ca/2013/07/critically-damped-spring-smoothing.html
            float ddPhaseOffset = smoothing * ((smoothing / 4.0f) * (tPhaseOffset[i] - phaseOffset[i]) - dPhaseOffset[i]); // 2nd derivative
            dPhaseOffset[i] = dPhaseOffset[i] + dt * ddPhaseOffset; // 1st derivative
            phaseOffset[i] = phaseOffset[i] + dt * dPhaseOffset[i];

            float ddAmp = smoothing * ((smoothing / 4.0f) * (tAmp[i] - amp[i]) - dAmp[i]); // 2nd derivative
            dAmp[i] = dAmp[i] + dt * ddAmp; // 1st derivative
            amp[i] = amp[i] + dt * dAmp[i];
        }
     
        i = 0;
        foreach (var j in shoulderJoints)
        {
            float osc = Mathf.Sin(cpg.phase[i] + PHASE_OFFSET[i]);
            
            osc = (1.0f / (1.0f + Mathf.Exp(-sig * osc)) - .5f) * 2.0f;

            var spring = j.spring;
            spring.targetPosition = Mathf.Lerp(j.limits.min, j.limits.max, .5f * (osc + 1.0f)) * amp[i];
            j.spring = spring;            
            i++;
        }

        i = 0;

        // legs stops moving if the shoulder does
        float a = 1 - Mathf.Exp(-4 * Mathf.Clamp01(Mathf.Max(Mathf.Abs(left), Mathf.Abs(right))));

        foreach (var j in kneeJoints)
        {
            float osc = Mathf.Sin(cpg.phase[i] * 1.0f + phaseOffset[i] * Mathf.PI / 2.0f);
            
            osc = (1.0f / (1.0f + Mathf.Exp(-sig * osc)) - .5f) * 2.0f;

            var spring = j.spring;

            spring.targetPosition = Mathf.Lerp(j.limits.min, j.limits.max, .5f * (osc + 1.0f)) * a;
            j.spring = spring;            
            i++;
        }
    }

    static void GetKneeJointPhaseOffset(float leftTrack, float rightTrack, ref float[] offset)
    {
        for (int i = 0; i < N_LEGS; ++i)
        {
            offset[i] = (i % 2 == 0 ? leftTrack : rightTrack);
        }        
    }

    static void GetShoulderJointAmp(float leftTrack, float rightTrack, ref float[] amp)
    {
        for (int i = 0; i < N_LEGS; ++i)
        {
            amp[i] = Mathf.Abs(i % 2 == 0 ? leftTrack : rightTrack);
        }        
    }
}
