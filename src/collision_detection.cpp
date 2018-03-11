#include "collision_detection.h"
#include "phys.h"
#include "../../SGV3D/src/logger.h"
#include <iterator>

void OverlapCache::SortCache () 
{
    std::sort(m_cache.begin(),m_cache.end());
}
  
void OverlapCache::SetPairForDelete (OverlappingPair const& delPair)
{
    ASSERT(m_deleteCount <= m_cache.size());

    auto lookup{std::find(m_cache.begin(), m_cache.end()-m_deleteCount, delPair)};
    if(lookup != m_cache.end()-m_deleteCount)
    {
        ++m_deleteCount;
        std::iter_swap(lookup, m_cache.end() - m_deleteCount);
    }
}

void OverlapCache::CleanUp ()
{
    ASSERT(m_deleteCount <= m_cache.size());

    m_cache.resize(m_cache.size() - m_deleteCount);
    m_deleteCount = 0;
}

void OverlapCache::AddPair (OverlappingPair const& addPair)
{
    auto lookup{std::find(m_cache.begin(), m_cache.end(), addPair)};
    if(lookup == m_cache.end())
        m_cache.insert(m_cache.begin(), addPair);
}

BroadPhase::~BroadPhase ()
{
    for(uint8_t axisIdx = 0; axisIdx < 3; ++axisIdx)
    {
        for(auto& endpoint: m_axes[axisIdx])
        {
            if(endpoint)
                delete endpoint;
        }
    }
}
    
Box const& BroadPhase::RetrieveBox (unsigned const& idx) const
{
    ASSERT(idx < m_boxes.size());
    return m_boxes[idx];
}

unsigned BroadPhase::InsertInternal (Endpoint* endpoint, unsigned const& offset, uint8_t axisIdx)
{
    auto& axis{m_axes[axisIdx]};
    ASSERT(offset <= axis.size());

    unsigned pos{offset};
    for(; pos < axis.size(); ++pos)
    {
        if(endpoint->Value() < axis[pos]->Value())
            break;
    }
    axis.insert(axis.begin()+pos, endpoint);
    return pos+1;
//    auto it{std::upper_bound(axis.begin() + offset, axis.end(), endpoint,
//            [](Endpoint* lhs, Endpoint* rhs){return *lhs < *rhs;})};
    
//    axis.insert(it, endpoint);
}

void BroadPhase::Insert (RigidBody* rb)
{
    Collidable* coll{rb->GetCollidable()};

    Endpoint* min[3]{new Endpoint, new Endpoint, new Endpoint};
    Endpoint* max[3]{new Endpoint, new Endpoint, new Endpoint};

    coll->CalculateBoundingBox(min, max, rb);
    
    Box box{min, max, rb};

    unsigned n{(unsigned)m_boxes.size()};
    m_boxes.push_back(box);

    for(uint8_t axisIdx = 0; axisIdx < 3; ++axisIdx)
    { 
        min[axisIdx]->SetIsMin(true);
        max[axisIdx]->SetIsMin(false);

        min[axisIdx]->SetBoxIndex(n);
        max[axisIdx]->SetBoxIndex(n);

        InsertInternal(min[axisIdx], 0, axisIdx);
        InsertInternal(max[axisIdx], 0, axisIdx);
    }
}

void BroadPhase::BatchInsert (std::vector<RigidBody*> const& rbs)
{
    std::vector<Endpoint*> endpoints[3];
    for(uint8_t axisIdx = 0; axisIdx < 3; ++axisIdx)
    {
        endpoints[axisIdx].resize(2*rbs.size());
    }

    for(unsigned i = 0; i < rbs.size(); ++i)
    {
        RigidBody* rb{rbs[i]};

        Collidable* coll{rb->GetCollidable()};

        Endpoint* min[3]{new Endpoint, new Endpoint, new Endpoint};
        Endpoint* max[3]{new Endpoint, new Endpoint, new Endpoint};

        coll->CalculateBoundingBox(min, max, rb);

        Box box{min, max, rb};

        endpoints[0][2*i] = std::move(min[0]);
        endpoints[0][2*i + 1] = std::move(max[0]);

        endpoints[1][2*i] = std::move(min[1]);
        endpoints[1][2*i + 1] = std::move(max[1]);

        endpoints[2][2*i] = std::move(min[2]);
        endpoints[2][2*i + 1] = std::move(max[2]);

        unsigned n{(unsigned)m_boxes.size()};
        m_boxes.push_back(box);

        for(uint8_t axisIdx = 0; axisIdx < 3; ++axisIdx)
        {
            endpoints[axisIdx][2*i]->SetIsMin(true);
            endpoints[axisIdx][2*i + 1]->SetIsMin(false);

            endpoints[axisIdx][2*i]->SetBoxIndex(n);
            endpoints[axisIdx][2*i + 1]->SetBoxIndex(n);
        }
    }
    
    for(uint8_t axisIdx = 0; axisIdx < 3; ++axisIdx)
    {
        auto& axisEndpoints{endpoints[axisIdx]};
        std::sort(axisEndpoints.begin(), axisEndpoints.end());

        unsigned insertPos{0};
        for(unsigned i = 0; i < axisEndpoints.size(); ++i)
        {
            insertPos = InsertInternal(axisEndpoints[i], insertPos, axisIdx);
        }
    }
}

void BroadPhase::InitialOverlapCache () 
{
    for(unsigned i = 0; i < m_boxes.size(); ++i)
    {
        for(unsigned j = i+1; j < m_boxes.size(); ++j)
        {
            Box const& bx1{m_boxes[i]}, bx2{m_boxes[j]};

            if(!(     bx1.max[0]->Value() < bx2.min[0]->Value() 
                  ||  bx2.max[0]->Value() < bx1.min[0]->Value() 
                  ||  bx1.max[1]->Value() < bx2.min[1]->Value() 
                  ||  bx2.max[1]->Value() < bx1.min[1]->Value()
                  ||  bx1.max[2]->Value() < bx2.min[2]->Value() 
                  ||  bx2.max[2]->Value() < bx1.min[2]->Value()))
            {
                m_overlappingPairs.AddPair({i,j});
            }
        }
    }
}

bool BroadPhase::OverlapCheck2D (OverlapCache::OverlappingPair const& pollPair, uint8_t axisIdx) const
{
    //Get other axes
    uint8_t axisIdx1 = (axisIdx+1)%3,
            axisIdx2 = (axisIdx+2)%3;

    Box const& bx1{m_boxes[pollPair.rbIdx1]}, bx2{m_boxes[pollPair.rbIdx2]};

    return !(     bx1.max[axisIdx1]->Value() < bx2.min[axisIdx1]->Value() 
              ||  bx2.max[axisIdx1]->Value() < bx1.min[axisIdx1]->Value() 
              ||  bx1.max[axisIdx2]->Value() < bx2.min[axisIdx2]->Value() 
              ||  bx2.max[axisIdx2]->Value() < bx1.min[axisIdx2]->Value());
}

std::vector<OverlapCache::OverlappingPair> const& BroadPhase::FindOverlappingBoxes ()
{
    //Update 
    for(Box& box: m_boxes)
    {
        Collidable* coll{box.rb->GetCollidable()};
        coll->CalculateBoundingBox(box.min, box.max, box.rb);
    }

    //Insertion sort
    for(uint8_t axisIdx = 0; axisIdx < 3; ++axisIdx)
    {
        auto& axis{m_axes[axisIdx]};
        for(unsigned i = 1; i < axis.size(); ++i)
        {
            for(unsigned j = i; j > 0; --j)
            {
                if(axis[j-1]->Value() <= axis[j]->Value() + FLT_EPSILON)
                    break;
                
                OverlapCache::OverlappingPair pollPair{axis[j-1]->GetBoxIndex(), axis[j]->GetBoxIndex()};
                if(pollPair.rbIdx1 == pollPair.rbIdx2)
                {
                    std::swap(axis[j-1],axis[j]);
                    continue;
                }

                if(axis[j-1]->IsMin() && axis[j]->IsMax())
                    m_overlappingPairs.SetPairForDelete(pollPair);

                else if(axis[j-1]->IsMax() && axis[j]->IsMin() && OverlapCheck2D(pollPair, axisIdx))
                    m_overlappingPairs.AddPair(pollPair);

                std::swap(axis[j-1],axis[j]);
            } 
        }
    }

    m_overlappingPairs.CleanUp();
    return m_overlappingPairs.GetCache();
}
