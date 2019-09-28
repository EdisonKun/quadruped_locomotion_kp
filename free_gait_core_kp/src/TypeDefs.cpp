#include "TypeDefs.hpp"
namespace free_gait {
struct CompareByCounterClockwiseOrder{
    bool operator() (const LimbEnum& a, const LimbEnum& b) const
    {
        const size_t aIndex = findIndex(a);
        const size_t bIndex = findIndex(b);
        return aIndex < bIndex;
    }

    size_t findIndex(const LimbEnum& limb) const
    {
        auto it =std::find(limbEnumCounterClockWiseOrder.begin(), limbEnumCounterClockWiseOrder.end(), limb);//between begin and end, find limb
        if (it == limbEnumCounterClockWiseOrder.end())
        {
            throw std::runtime_error("Could not find index for limb!");
        }
        return std::distance(limbEnumCounterClockWiseOrder.begin(), it);//the distance between begin and it
    }
};

void getFootholdsCounterClockwiseOrdered(const Stance& stance, std::vector<Position>& footholds)
{
    footholds.reserve(stance.size());
    std::map<LimbEnum, Position, CompareByCounterClockwiseOrder> stanceOrdered;//the third one is a struct
    for (const auto& limb : stance) stanceOrdered.insert(limb);
    for (const auto& limb : stanceOrdered) footholds.push_back(limb.second);
};

}//namespace
