using UnityEngine;

public class FpsDisplay : MonoBehaviour
{
    private float deltaTime = 0.0f;
    public string sysMessage = "Status: Initialising";
    public string debugMessage = "";
    public bool showFPS = false;

    private void Update()
    {
        deltaTime += (Time.unscaledDeltaTime - deltaTime) * 0.1f;
    }

    private void OnGUI()
    {
        int fps = Mathf.RoundToInt(1.0f / deltaTime);
        // string text = $"FPS: {fps}";
        string fpsText = showFPS ? $"FPS: {fps}" : "";
        string text = $"{sysMessage}\n{fpsText}\n{debugMessage}";

        GUIStyle style = new GUIStyle();
        style.normal.textColor = Color.white;
        style.alignment = TextAnchor.UpperRight;
        style.fontSize = 20;

        // Display the FPS on the top right corner of the screen
        GUI.Label(new Rect(Screen.width-110, 10, 100, 20), text, style);
    }
}