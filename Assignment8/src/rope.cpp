#include <iostream>
#include <vector>

#include "CGL/vector2D.h"

#include "mass.h"
#include "rope.h"
#include "spring.h"

namespace CGL {

    Rope::Rope(Vector2D start, Vector2D end, int num_nodes, float node_mass, float k, vector<int> pinned_nodes)
    {
        // TODO (Part 1): Create a rope starting at `start`, ending at `end`, and containing `num_nodes` nodes.

//        Comment-in this part when you implement the constructor
//        for (auto &i : pinned_nodes) {
//            masses[i]->pinned = true;
//        }
        Vector2D ropePace = (end-start) / (num_nodes-1);
        Mass* temp = new Mass(start, node_mass, false);
        Mass* last = temp;
        masses.push_back(temp);
        Vector2D pos = start;

        for(int i=1; i < num_nodes; ++i)
        {
            pos += ropePace;
            temp = new Mass(pos, node_mass, false);
            masses.push_back(temp);

            Spring* s = new Spring(last, temp, k);
            springs.push_back(s);

            last = temp;
        }

        for(auto& i: pinned_nodes){
            masses[i]->pinned = true;
        }

    }

    void Rope::simulateEuler(float delta_t, Vector2D gravity)
    {
        for (auto &s : springs)
        {
            // TODO (Part 2): Use Hooke's law to calculate the force on a node
            Mass* a = s->m1, *b = s->m2;
            Vector2D a2b = b->position - a->position;
            Vector2D a2bForce = a2b / a2b.norm() * s->k * (a2b.norm() - s->rest_length);
            a->forces += a2bForce;
            b->forces -= a2bForce;
        }

        for (auto &m : masses)
        {
            double k_d = 0.005;
            if (!m->pinned)
            {
                // TODO (Part 2): Add the force due to gravity, then compute the new velocity and position
                m->forces += gravity * m->mass;
                m->forces -= k_d * m->velocity;
                Vector2D a = m->forces / m->mass;
                m->velocity = m->velocity + delta_t * a;
                m->position = m->position + m->velocity * delta_t;
                // TODO (Part 2): Add global damping
            }

            // Reset all forces on each mass
            m->forces = Vector2D(0, 0);
        }
    }

    void Rope::simulateVerlet(float delta_t, Vector2D gravity)
    {
        for (auto &s : springs)
        {
            // TODO (Part 3): Simulate one timestep of the rope using explicit Verlet （solving constraints)
            Mass* a = s->m1, *b = s->m2;
            Vector2D a2b = b->position - a->position;
            Vector2D a2bForce = a2b / a2b.norm() * s->k * (a2b.norm() - s->rest_length);
            a->forces += a2bForce;
            b->forces -= a2bForce;
            
        }
        double damping = 0.00005;
        for (auto &m : masses)
        {
            if (!m->pinned)
            {
                Vector2D temp_position = m->position;
                // TODO (Part 3.1): Set the new position of the rope mass
                m->forces += gravity * m->mass;
                Vector2D a = m->forces / m->mass;
                m->position = m->position + (1-damping) * (m->position - m->last_position) + a * delta_t * delta_t;
                m->last_position = temp_position;
                // TODO (Part 4): Add global Verlet damping
            }
        }
    }
}
