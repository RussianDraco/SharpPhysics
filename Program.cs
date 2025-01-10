using System;
using System.Collections.Generic;
using System.Numerics;

namespace SharpPhysics;

public class PhysicsBody {
    public Vector2 Position;
    public Vector2 Velocity;
    public Vector2 Acceleration;
    public float Mass;

    public PhysicsBody(Vector2 position, float mass) {
        Position = position;
        Velocity = Vector2.Zero;
        Acceleration = Vector2.Zero;
        Mass = mass;
    }

    public void ApplyForce(Vector2 force) {
        Acceleration += force * (1 / Mass);
    }

    public void Update(float deltaTime) {
        Velocity += Acceleration * deltaTime;
        Position += Velocity * deltaTime;
        Acceleration = Vector2.Zero;
    }
}

public class CollisionDetector {
    public static bool CheckCollision(PhysicsBody body1, PhysicsBody body2, float radius) {
        float distance = Vector2.Distance(body1.Position, body2.Position);
        return distance < radius * 2;
    }
    public static void ResolveCollision(PhysicsBody body1, PhysicsBody body2) {
        Vector2 normal = Vector2.Normalize(body1.Position - body2.Position);
        Vector2 relativeVelocity = body1.Velocity - body2.Velocity;
        float velocityAlongNormal = Vector2.Dot(relativeVelocity, normal);
        if (velocityAlongNormal > 0) {
            return;
        }
        float restitution = 0.8f;
        float impulse = -(1 + restitution) * velocityAlongNormal;
        impulse /= 1 / body1.Mass + 1 / body2.Mass;
        Vector2 impulseVector = impulse * normal;
        body1.Velocity += 1 / body1.Mass * impulseVector;
        body2.Velocity -= 1 / body2.Mass * impulseVector;
    }
}

public class PhysicsWorld {
    public List<PhysicsBody> Bodies = new List<PhysicsBody>();
    public float Gravity = 9.81f;

    public void AddBody(PhysicsBody body) {
        Bodies.Add(body);
    }

    public void Step(float deltaTime) {
        foreach (var body in Bodies) {
            body.ApplyForce(new Vector2(0, -Gravity * body.Mass));
            body.Update(deltaTime);
        }

        for (int i = 0; i < Bodies.Count; i++) {
            for (int j = i + 1; j < Bodies.Count; j++) {
                if (CollisionDetector.CheckCollision(Bodies[i], Bodies[j], 1)) {
                    CollisionDetector.ResolveCollision(Bodies[i], Bodies[j]);
                }
            }
        }
    }
}

class Program {
    static void Main(string[] args) {
        PhysicsWorld world = new PhysicsWorld();
        
        PhysicsBody body1 = new PhysicsBody(new Vector2(0, 0), 1);
        PhysicsBody body2 = new PhysicsBody(new Vector2(0, 2), 1);

        world.AddBody(body1);
        world.AddBody(body2);

        float deltaTime = 0.016f;

        for (int i = 0; i < 1000; i++) {
            world.Step(deltaTime);
            Console.WriteLine($"Body1: {body1.Position} Body2: {body2.Position}");
        }
    }
}