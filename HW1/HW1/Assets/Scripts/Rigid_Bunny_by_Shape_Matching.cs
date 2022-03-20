using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Rigid;


public class Rigid_Bunny_by_Shape_Matching : MonoBehaviour
{
	public bool launched = false;
	Vector3[] X;
	Vector3[] Q;
	Vector3[] V;
	Matrix4x4 QQt = Matrix4x4.zero;

	float dt = 0.015f;
	float linear_decay = 0.999f;
	float restitution = 0.5f;
	float u_t = 0.5f;
	float coeff = 1.3f;

	Vector3[] P = { new Vector3(0, 0.01f, 0), new Vector3(2, 0, 0) };
	Vector3[] N = { new Vector3(0, 1, 0), new Vector3(-1, 0, 0) };

	// for collision（for penalty)
	Vector3[] force;
	//施加力的系数
	float tho = 10;
	//开始作用的距离
	float epsilon = 0.03f;


	// Start is called before the first frame update
	void Start()
    {
    	Mesh mesh = GetComponent<MeshFilter>().mesh;
        V = new Vector3[mesh.vertices.Length];
        X = mesh.vertices;
        Q = mesh.vertices;

        //Centerizing Q.
        Vector3 c=Vector3.zero;
        for(int i=0; i<Q.Length; i++)
        	c+=Q[i];
        c/=Q.Length;
        for(int i=0; i<Q.Length; i++)
        	Q[i]-=c;

        //Get QQ^t ready.
		for(int i=0; i<Q.Length; i++)
		{
			QQt[0, 0]+=Q[i][0]*Q[i][0];
			QQt[0, 1]+=Q[i][0]*Q[i][1];
			QQt[0, 2]+=Q[i][0]*Q[i][2];
			QQt[1, 0]+=Q[i][1]*Q[i][0];
			QQt[1, 1]+=Q[i][1]*Q[i][1];
			QQt[1, 2]+=Q[i][1]*Q[i][2];
			QQt[2, 0]+=Q[i][2]*Q[i][0];
			QQt[2, 1]+=Q[i][2]*Q[i][1];
			QQt[2, 2]+=Q[i][2]*Q[i][2];
		}
		QQt[3, 3]=1;

		for(int i=0; i<X.Length; i++)
			V[i][0]=4.0f;

		Update_Mesh(transform.position, Matrix4x4.Rotate(transform.rotation), 0);
		transform.position=Vector3.zero;
		transform.rotation=Quaternion.identity;
   	}

   
	// Update the mesh vertices according to translation c and rotation R.
	// It also updates the velocity.
	void Update_Mesh(Vector3 c, Matrix4x4 R, float inv_dt)
   	{
   		for(int i=0; i<Q.Length; i++)
		{
			Vector3 x=(Vector3)(R*Q[i])+c;

			V[i]+=(x-X[i])*inv_dt;
			X[i]=x;
		}	
		Mesh mesh = GetComponent<MeshFilter>().mesh;
		mesh.vertices=X;
   	}

	void Collision(float inv_dt)
	{
		for (int i = 0; i < 2; i++)
		{
			for (int j = 0; j < V.Length; j++)
			{
				Vector3 xj = X[j];
				Vector3 Pxj = xj - P[i];
				float ProjX = Vector3.Dot(Pxj, N[i]);
				if (ProjX < epsilon)
				{
					float v_n_num = Vector3.Dot(V[j], N[i]);
					if (v_n_num < 0)
					{
						Vector3 v_n = v_n_num * N[i];
						Vector3 v_t = V[j] - v_n;
						Vector3 vNnew = -restitution * v_n;
						float a = Mathf.Max(0, 1 - u_t * (1 + restitution) * v_n.magnitude / v_t.magnitude);
						Vector3 vTnew = a * v_t;
						V[j] = vNnew + vTnew;
						X[j] = X[j] + Mathf.Abs(ProjX) * N[i] * coeff;
						restitution *= 0.5f;
					}
				}
			}
		}
	}

    // Update is called once per frame
    void Update()
    {
		if (Input.GetKey("r"))
		{
			transform.position = new Vector3(0, 0.6f, 0);
			restitution = 0.5f;
			launched = false;
		}
		if (Input.GetKey("l"))
		{
			launched = true;
		}
		if (!launched) return;

  		//Step 1: run a simple particle system.
        for(int i = 0; i < V.Length; i++)
        {
			V[i] = linear_decay * (V[i] +  MathUtil.gravity * dt);
			X[i] = X[i] + V[i] * dt;
		}

        //Step 2: Perform simple particle collision.
		Collision(1/dt);

		// Step 3: Use shape matching to get new translation c and 
		// new rotation R. Update the mesh by c and R.
		//Shape Matching (translation)
		Vector3 c = new Vector3(0, 0, 0);
		for (int i = 0; i < V.Length; i++)
		{
			c = c + X[i];
		}
		c = c / V.Length;
		//Shape Matching (rotation)
		Matrix4x4 A = Matrix4x4.zero;
		for (int i = 0; i < Q.Length; i += 5)
		{
			Vector3 Ri = X[i] - c;
			A[0, 0] += Ri[0] * Q[i][0] * 5;
			A[0, 1] += Ri[0] * Q[i][1] * 5;
			A[0, 2] += Ri[0] * Q[i][2] * 5;
			A[1, 0] += Ri[1] * Q[i][0] * 5;
			A[1, 1] += Ri[1] * Q[i][1] * 5;
			A[1, 2] += Ri[1] * Q[i][2] * 5;
			A[2, 0] += Ri[2] * Q[i][0] * 5;
			A[2, 1] += Ri[2] * Q[i][1] * 5;
			A[2, 2] += Ri[2] * Q[i][2] * 5;
		}
		A[3, 3] = 1;
		Matrix4x4 R = MathUtil.GetRotation(A * QQt.inverse);

		Update_Mesh(c, R, 1/dt);
	}
}
