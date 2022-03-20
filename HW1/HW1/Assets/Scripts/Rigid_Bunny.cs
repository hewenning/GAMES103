using UnityEngine;
using System.Collections;
using Rigid;

public class Rigid_Bunny : MonoBehaviour 
{
	bool launched 		= false;
	float dt 			= 0.015f;
	Vector3 v 			= new Vector3(0, 0, 0);	// velocity
	Vector3 w 			= new Vector3(0, 0, 0);	// angular velocity
	
	float mass;									// mass
	Matrix4x4 I_ref;							// reference inertia

	float linear_decay	= 0.999f;				// for velocity decay
	float angular_decay	= 0.98f;				
	float restitution 	= 0.9f;                 // for collision
	float u_t = 0.5f;
	Vector3[] vertices;

	// Use this for initialization
	void Start () 
	{		
		Mesh mesh = GetComponent<MeshFilter>().mesh;
		vertices = mesh.vertices;

		float m=1;
		mass=0;
		for (int i=0; i<vertices.Length; i++) 
		{
			mass += m;
			float diag=m*vertices[i].sqrMagnitude;
			I_ref[0, 0]+=diag;
			I_ref[1, 1]+=diag;
			I_ref[2, 2]+=diag;
			I_ref[0, 0]-=m*vertices[i][0]*vertices[i][0];
			I_ref[0, 1]-=m*vertices[i][0]*vertices[i][1];
			I_ref[0, 2]-=m*vertices[i][0]*vertices[i][2];
			I_ref[1, 0]-=m*vertices[i][1]*vertices[i][0];
			I_ref[1, 1]-=m*vertices[i][1]*vertices[i][1];
			I_ref[1, 2]-=m*vertices[i][1]*vertices[i][2];
			I_ref[2, 0]-=m*vertices[i][2]*vertices[i][0];
			I_ref[2, 1]-=m*vertices[i][2]*vertices[i][1];
			I_ref[2, 2]-=m*vertices[i][2]*vertices[i][2];
		}
		I_ref [3, 3] = 1;
	}

	// In this function, update v and w by the impulse due to the collision with
	//a plane <P, N>
	void Collision_Impulse(Vector3 P, Vector3 N)
	{
		Matrix4x4 R = Matrix4x4.Rotate(transform.rotation);
		Vector3 sumP = new Vector3(0, 0, 0);
		Vector3 sumV = new Vector3(0, 0, 0);
		int collisionCount = 0;
		Vector3 position = transform.position;

		for (int i =0; i < vertices.Length; i++)
        {
			// 判断位置
			Vector3 Rri = R * vertices[i];
			Vector3 x_i = position + Rri;
			Vector3 dis = x_i - P;
			float signed_dis = Vector3.Dot(dis, N);
			if(signed_dis < 0)
            {
				// 距离函数为负，判断法向速度
				Vector3 wxRi = MathUtil.GetCrossMatrix(w) * Rri;
				Vector3 v_i = v + wxRi;
				if(Vector3.Dot(v_i, N) < 0)
                {
					sumP = sumP + Rri;
					sumV = sumV + v_i;
					collisionCount ++;
				}
			}
			
        }
		if (collisionCount == 0) return;

		// 平均位移和平均速度
		Vector3 averageP = sumP / collisionCount;
		Vector3 averageV = sumV / collisionCount;

		// 碰撞前法线和切线速度
		Vector3 v_n = Vector3.Dot(averageV, N) * N;
		Vector3 v_t = averageV - v_n;
		// 碰撞后法线速度
		Vector3 v_n_new = -v_n * restitution;
		// 切线速度衰减
		float a = Mathf.Max(0, 1 - u_t * (1 + restitution) * v_n.sqrMagnitude / v_t.sqrMagnitude);
		Vector3 v_t_new = a * v_t;
		Vector3 v_new = v_n_new + v_t_new;

		// 解算冲量 j
		Matrix4x4 I = Matrix4x4.identity;
		Matrix4x4 CRri = MathUtil.GetCrossMatrix(averageP);
		Matrix4x4 K = MathUtil.Subtract(MathUtil.Scale(I, 1.0f/ mass), CRri * I_ref.inverse * CRri);
		Vector3 j = K.inverse * (v_new - averageV);

		// 更新 v 和 w
		v = v + 1.0f / mass * j;
		Vector3 w_delta = I_ref.inverse * CRri * j;
		w = w + w_delta;
		restitution = restitution * 0.5f;
	}

	// Update is called once per frame
	void Update () 
	{
		//Game Control
		if(Input.GetKey("r"))
		{
			transform.position = new Vector3 (0, 0.6f, 0);
			restitution = 0.5f;
			launched=false;
		}
		if(Input.GetKey("l"))
		{
			v = new Vector3 (5, 2, 0);
			launched=true;
		}
		if (!launched) return;

		// Part I: Update velocities
		// 重力
		v = v +  MathUtil.gravity * dt;
		// 衰减
		v = v * linear_decay;
		w = w * angular_decay;

		// Part II: Collision Impulse
		Collision_Impulse(new Vector3(0, 0.01f, 0), new Vector3(0, 1, 0));
		Collision_Impulse(new Vector3(2, 0, 0), new Vector3(-1, 0, 0));

		// Part III: Update position & orientation
		// Update linear status
		Vector3 x = transform.position;
		x = x + dt * v;

		// Update angular status
		Quaternion q = transform.rotation;
		Vector3 vq = new Vector3(q.x, q.y, q.z);
		Vector3 vw = w * dt / 2;
		Vector3 temp = MathUtil.GetCrossMatrix(vw) * vq;
		Vector3 svq = (q.w * vw) + temp;
		Quaternion q_new = new Quaternion(vq[0] + svq[0], vq[1] + svq[1], vq[2] + svq[2], q.w -Vector3.Dot(vw, vq));

		// Part IV: Assign to the object
		transform.position = x;
		transform.rotation = q_new;
	}
}