using UnityEngine;
using System.Collections;

public class AiTesterPlayer : MonoBehaviour {

    [Range(0.1f,1.0f)]
    public float playerSpeed;

	// Update is called once per frame
	void Update ()
    {
        // Super basic movement
        if (Input.GetKey(KeyCode.W))
            transform.position += Vector3.forward * playerSpeed /5;
        if (Input.GetKey(KeyCode.S))
            transform.position -= Vector3.forward * playerSpeed /5;

        if (Input.GetKey(KeyCode.D))
            transform.position += Vector3.right* playerSpeed /5;
        if (Input.GetKey(KeyCode.A))
            transform.position -= Vector3.right* playerSpeed /5;
    }
}
