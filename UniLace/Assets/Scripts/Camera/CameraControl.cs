using UnityEngine;

public class CameraControl : MonoBehaviour
{
    public float moveSpeed = 5f;
    public float rotationSpeed = 100f;
    public bool isTransformLocked = true; // Flag to determine if transform is locked

    private void Update()
    {      
        if (!isTransformLocked)
        {
            // Move the game object based on keyboard input
            float horizontalInput = Input.GetAxis("Horizontal");
            float verticalInput = Input.GetAxis("Vertical");
            float upInput = Input.GetKey(KeyCode.Space) ? 1f : 0f;
            float downInput = Input.GetKey(KeyCode.LeftShift) ? -1f : 0f;

            Vector3 movement = new Vector3(horizontalInput, upInput + downInput, verticalInput) * moveSpeed * Time.deltaTime;
            transform.parent.transform.Translate(movement, Space.Self);

            // Rotate the game object based on second player input
            float horizontal2Input = Input.GetAxis("Horizontal2");
            float vertical2Input = Input.GetAxis("Vertical2");
            Vector3 pitch = new Vector3(-vertical2Input, 0, 0f) * rotationSpeed * Time.deltaTime;
            transform.Rotate(pitch, Space.Self);
            Vector3 yaw = new Vector3(0f, horizontal2Input, 0f) * rotationSpeed * Time.deltaTime;
            transform.parent.transform.Rotate(yaw, Space.Self);
        }
    }
    // Enter/Esc button to lock/unlock the transform
    private void OnGUI()
    {
        if (Event.current.type == EventType.KeyDown && Event.current.keyCode == KeyCode.Return)
        {
            isTransformLocked = !isTransformLocked;
        }
        if (Event.current.type == EventType.KeyDown && Event.current.keyCode == KeyCode.Q)
        {
            isTransformLocked = !isTransformLocked;
        }
    } 
}
