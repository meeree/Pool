#include "collidable.h"
#include "phys.h"
#include "collision_detection.h"
#include <unordered_map>

void SphereCollidable::CalculateBoundingBox(Endpoint* (&min)[3], Endpoint* (&max)[3], RigidBody* rb) 
{
    arma::vec3 pos{rb->Position()};

    for(uint8_t axisIdx = 0; axisIdx < 3; ++axisIdx)
    {
        if(min[axisIdx])
            min[axisIdx]->SetValue(pos[axisIdx] - m_rad);

        if(max[axisIdx])
            max[axisIdx]->SetValue(pos[axisIdx] + m_rad);
    }
}

HalfEdgeMesh::HalfEdgeMesh (std::vector<arma::vec3> const& positions, std::vector<std::vector<unsigned>> const& faces)
{
    size_t vert_cnt{0};
    for(auto const& face: faces)
    {
        vert_cnt += face.size();
    }

	m_positions.resize(positions.size());
	std::copy(positions.begin(), positions.end(), m_positions.begin());
    m_vert_lookup.resize(positions.size());

    m_vertices.resize(vert_cnt);
    m_faces.resize(faces.size());
    m_half_edges.resize(vert_cnt);
    
    static const auto hash_func{[](std::pair<unsigned, unsigned> const& p){return p.first ^ p.second;} };
    std::unordered_map<std::pair<unsigned, unsigned>, HalfEdge*, decltype(hash_func)> edge_hash(vert_cnt, hash_func);

    auto face_it = m_faces.begin();
    auto vert_it = m_vertices.begin();
    auto edge_it = m_half_edges.begin();
    for(auto const& face: faces)
    {
        //Outward pointing normal. Faces are ccw. 
        ASSERT(face.size() >= 3);
        arma::vec3 const& v0{positions[face[0]]};
        arma::vec3 const& v1{positions[face[1]]};
        arma::vec3 const& v2{positions[face[2]]};
        arma::vec3 normal = arma::cross(v1 - v0, v2 - v1);

        double const nm = arma::norm(normal);
        ASSERT(nm > DBL_EPSILON);
        normal /= nm;

        face_it->normal = std::move(normal);

        auto vert_it_begin = vert_it;
        for(auto const& idx: face)
        {
            vert_it->idx= idx;
            m_vert_lookup[idx].push_back(&(*vert_it));

            ++vert_it;
        }

        unsigned idx{0};
        auto vert_it_end = vert_it;
        auto edge_it_begin = edge_it;
        --vert_it;
        for(; idx < face.size(); ++idx, --vert_it, ++edge_it)
        {
            edge_it->end_vert = &(*vert_it);
            edge_it->face = &(*face_it);
            
            if(edge_it != edge_it_begin)
            {
                edge_it->next_edge = &(*(edge_it - 1));
                vert_it->edge = edge_it->next_edge;
            }
        }

        edge_it_begin->next_edge = &(*(edge_it-1)); 
        (vert_it_end-1)->edge = edge_it_begin->next_edge;

        //Setup opposite edges
        auto edge_it_end = edge_it;
        edge_it = edge_it_begin;
        auto edge_it_prev = edge_it_end - 1;
        for(; edge_it != edge_it_end; ++edge_it)
        {
            // Pair is (end, start). We flip for lookup of opposite edge.  
            std::pair<unsigned, unsigned> lookup{edge_it->end_vert->idx, edge_it_prev->end_vert->idx};
            if(edge_hash.find(lookup) != edge_hash.end())
            {
                HalfEdge* opposite_edge{edge_hash[lookup]};
                edge_it->opposite_edge = opposite_edge;
                opposite_edge->opposite_edge = &(*(edge_it));
            }
            else
            {
                edge_hash[std::make_pair(edge_it_prev->end_vert->idx, edge_it->end_vert->idx)] = &(*(edge_it));
                edge_it->opposite_edge = nullptr;
            }

            edge_it_prev = edge_it;
        } 

        vert_it = vert_it_end;

        face_it->edge = &(*(edge_it_begin));
        ++face_it;
    } 
}

void MeshInstance::Update (arma::vec3 const& trans, arma::mat33 const& ortho_mat)
{
    std::vector<arma::vec3> const& positions{m_mesh->VertexPositions()};
    std::vector<HalfEdgeMesh::Face> const& faces{m_mesh->Faces()};

    std::transform(positions.begin(), positions.end(), m_transformed_positions.begin(),
            [trans, ortho_mat](arma::vec3 const& pos){return trans + ortho_mat * pos;});

    std::transform(faces.begin(), faces.end(), m_transformed_normals.begin(),
            [ortho_mat](HalfEdgeMesh::Face const& face){return ortho_mat * face.normal;});
}

void MeshCollidable::CalculateBoundingBox(Endpoint* (&min)[3], Endpoint* (&max)[3], RigidBody* rb) 
{
    for(uint8_t axisIdx = 0; axisIdx < 3; ++axisIdx)
    {
        if(min[axisIdx])
            min[axisIdx]->SetValue(DBL_MAX);

        if(max[axisIdx])
            max[axisIdx]->SetValue(-DBL_MAX);
    }

    //Update instance with position and prientation or rigid body
    arma::vec3 const& pos{rb->Position()};
    arma::mat33 const& orientation{rb->Orientation()};
    UpdateInstance(pos, orientation);

    for(arma::vec3 vert_pos: TransformedVertexPositions())
    {
        for(uint8_t axisIdx = 0; axisIdx < 3; ++axisIdx)
        {
            double const& p{vert_pos[axisIdx]};

            if(min[axisIdx] && p < min[axisIdx]->Value())
                min[axisIdx]->SetValue(p);
            if(max[axisIdx] && p > max[axisIdx]->Value())
                max[axisIdx]->SetValue(p);
        }
    }
}
