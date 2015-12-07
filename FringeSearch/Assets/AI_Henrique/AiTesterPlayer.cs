using UnityEngine;
using System.Collections;

public class AiTesterPlayer : MonoBehaviour {

    public float playerSpeed = 15.0f;

    Rigidbody MyRigidBody;
	
    // Use this for initialization
	void Start ()
    {
        MyRigidBody = GetComponent<Rigidbody>();
    }
	
	// Update is called once per frame
	void Update ()
    {
        // Super basic movement
        if (Input.GetKey(KeyCode.W)) MyRigidBody.velocity = new Vector3(MyRigidBody.velocity.x,0,playerSpeed);
        if (Input.GetKey(KeyCode.S)) MyRigidBody.velocity = new Vector3(MyRigidBody.velocity.x, 0, -playerSpeed);

        if (Input.GetKey(KeyCode.D)) MyRigidBody.velocity = new Vector3(playerSpeed, 0, MyRigidBody.velocity.z);
        if (Input.GetKey(KeyCode.A)) MyRigidBody.velocity = new Vector3(-playerSpeed, 0, MyRigidBody.velocity.z);


    }
}
