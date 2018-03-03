#ifndef  __COLLISION_DETECTION_H__
#define  __COLLISION_DETECTION_H__

#include "core.h"
//Code based on (but quite different to) overview in this paper: http://www.codercorner.com/SAP.pdf

class RigidBody;
struct Box;

struct Endpoint
{
private:
    float m_value;
    unsigned m_data; //concatenated "isMin" bit and box index. See link above.

public:

    Endpoint () : m_data{0} {}
    Endpoint (float const& value, bool const& isMin) : m_value{value}, m_data{0} {SetIsMin(isMin);}

    inline bool IsMin () const {return (m_data >> 31) & 1U;}
    inline bool IsMax () const {return !IsMin();}
    inline void SetIsMin (bool isMin) {m_data |= (isMin ? 2147483648 : 0);}

    inline float const& Value () const {return m_value;}
    inline void SetValue (float const& value) {m_value = value;}

    inline unsigned GetBoxIndex () const {return m_data & 2147483647;}
    inline void SetBoxIndex (unsigned const& idx) {m_data |= (idx & 2147483647);}

    inline bool operator< (Endpoint const& rhs) {return m_value < rhs.m_value;}
};

struct Box 
{
    Box () : min{nullptr, nullptr, nullptr},  max{nullptr, nullptr, nullptr}, rb{nullptr} {}
    Box(Endpoint* const (&min_)[3], Endpoint* const (&max_)[3], RigidBody* rb_) : min{min_[0], min_[1], min_[2]}, max{max_[0], max_[1], max_[2]}, rb{rb_} {}

    Endpoint* min[3]; 
    Endpoint* max[3];
    RigidBody* rb;
};

class OverlapCache 
{
public:
    struct OverlappingPair 
    {
        unsigned rbIdx1, rbIdx2;
        OverlappingPair () {}
        OverlappingPair (unsigned const& rbIdx1_, unsigned const& rbIdx2_) : rbIdx1{std::min(rbIdx1_,rbIdx2_)}, rbIdx2{std::max(rbIdx1_,rbIdx2_)} {}

        bool operator< (OverlappingPair const& rhs) const
        {
            return rbIdx1 < rhs.rbIdx1 && rbIdx2 < rhs.rbIdx2;
        }

        bool operator== (OverlappingPair const& rhs) const
        {
            return rbIdx1 == rhs.rbIdx1 && rbIdx2 == rhs.rbIdx2;
        }
    };
    unsigned m_deleteCount;
private:
    std::vector<OverlappingPair> m_cache;
public:
    OverlapCache () : m_deleteCount{0} {}

    void SetPairForDelete (OverlappingPair const& delPair);
    void CleanUp ();

    inline void AddPair (OverlappingPair const& addPair);
    inline void FullClear () {m_cache.clear();}
    void SortCache ();
    inline std::vector<OverlappingPair> const& GetCache () const {return m_cache;}
};

class BroadPhase 
{
private:
    std::vector<Box> m_boxes;
    std::vector<Endpoint*> m_axes[3];
    OverlapCache m_overlappingPairs;

    //Insert endpoint and return index 
    unsigned InsertInternal (Endpoint* endpoint, unsigned const& offset, uint8_t axisIdx);

    bool OverlapCheck2D (OverlapCache::OverlappingPair const& pollPair, uint8_t axisIdx) const;

public:
    BroadPhase () {}
    ~BroadPhase ();

    //Note: batch inserts are a lot faster than singular insertions
    void Insert (RigidBody* rb);

    void InitialOverlapCache ();

    void BatchInsert (std::vector<RigidBody*> const& rbs);
    Box const& RetrieveBox (unsigned const& idx) const;

    std::vector<OverlapCache::OverlappingPair> const& FindOverlappingBoxes ();
};

#endif //__COLLISION_DETECTION_H__
